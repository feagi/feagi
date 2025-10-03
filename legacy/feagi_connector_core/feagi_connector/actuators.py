#!/usr/bin/env python3
from collections import deque
from feagi_connector import pns_gateway as pns
from feagi_connector import feagi_interface as feagi

actuators_mapped = {}
motor_data = dict()  # formerly rolling_window
capabilities = dict()  # This will be updated by a controller. On this file, it will initialize as an empty capabilities
servo_status = {}  # Formerly runtime['servo_status']
previous_servo_data = {}


def window_average(sequence):
    return sum(sequence) // len(sequence)

def motor_generate_power(power_maximum, feagi_power):
    return power_maximum * feagi_power


def start_motors(controller_capabilities):
    global actuators_mapped, motor_data, capabilities
    # if check_actuator_in_capabilities('motor'): # This needs to be after
    if 'motor' in controller_capabilities['output']:
        capabilities = controller_capabilities
        motor_data = dict()
        for motor_id in capabilities['output']['motor']:
            if 'rolling_window_len' in capabilities['output']['motor'][motor_id]:
                length_rolling_window = capabilities['output']['motor'][motor_id]['rolling_window_len']
            else:
                length_rolling_window = 0  # Default to 0 which will be extremely sensitive and stiff
            motor_data = create_motor_rolling_window_len(length_window=length_rolling_window,
                                                         current_rolling_window_dict=motor_data,
                                                         motor_id=motor_id)
        actuators_mapped = actuator_to_feagi_map(capabilities)


def start_servos(controller_capabilities):
    global actuators_mapped, capabilities
    capabilities = controller_capabilities
    for servo_id in capabilities['output']['servo']:
        if 'default_value' in capabilities['output']['servo'][servo_id]:
            if not int(servo_id) in servo_status:
                servo_status[int(servo_id)] = capabilities['output']['servo'][servo_id]['default_value']
    actuators_mapped = actuator_to_feagi_map(capabilities)

def start_generic_opu(controller_capabilities):
    global actuators_mapped, capabilities
    capabilities = controller_capabilities
    actuators_mapped = actuator_to_feagi_map(capabilities)



def feagi_id_converter(id):
    """
    This function converts motor IDs from 1,3,5,7 to 0,1,2,3.
    so, if you select 0 and 1, it will end up 0. if you select 2 and 3, it will end up 1.
    """
    if id % 2 == 0:
        return id // 2
    else:
        return (id - 1) // 2


def power_convert(motor_id, power):
    if motor_id % 2 == 0:
        return abs(power)
    else:
        return -1 * power


# Motor section
def convert_feagi_to_motor_opu(obtained_data):
    motor_from_feagi_data = dict()
    if 'motor' in obtained_data:
        if obtained_data['motor'] is not {}:
            sorted_keys = sorted(obtained_data['motor'].keys(), key=int)
            for data_point in sorted_keys:
                device_power = power_convert(data_point, obtained_data['motor'][data_point])
                device_id = feagi_id_converter(data_point)
                if device_id in motor_from_feagi_data:
                    motor_from_feagi_data[device_id] = motor_from_feagi_data[device_id] - obtained_data['motor'][
                        data_point]
                else:
                    motor_from_feagi_data[device_id] = device_power
    return motor_from_feagi_data


def update_power_of_motor(motor_from_feagi_data):
    global motor_data, capabilities
    if motor_from_feagi_data:
        for motor_id in motor_from_feagi_data:
            if str(motor_id) in capabilities['output']['motor']:
                if not capabilities['output']['motor'][str(motor_id)]['disabled']:
                    pass_the_power_to_motor(capabilities['output']['motor'][str(motor_id)]['max_power'],
                                            motor_from_feagi_data[motor_id],
                                            motor_id,
                                            motor_data)
    else:
        motor_data = rolling_window_update(motor_data)


def preparing_motor_data_to_controller():
    global actuators_mapped, motor_data
    send_motor_data_to_controller = dict()
    for motor_id in actuators_mapped['motor']:
        device_id_list = feagi_mapped_to_dev_index(dev_id='motor', feagi_index=motor_id, mapped_dict=actuators_mapped)
        for motor in device_id_list:
            data_power = motor_data[motor_id][0]
            send_motor_data_to_controller[motor] = data_power
    return send_motor_data_to_controller


def get_motor_data(feagi_data):
    if check_actuator_in_capabilities('motor'):
        converted_data = convert_feagi_to_motor_opu(obtained_data=feagi_data)
        update_power_of_motor(motor_from_feagi_data=converted_data)
        return preparing_motor_data_to_controller()
    else:
        return {}


def pass_the_power_to_motor(power_maximum, device_power, device_id, moving_average_len):
    device_power = int(motor_generate_power(power_maximum, device_power))
    if device_id in moving_average_len:
        moving_average_len = update_moving_average(moving_average_len, device_id, device_power)
    moving_average_len[device_id].append(window_average(moving_average_len[device_id]))
    moving_average_len[device_id].popleft()


def rolling_window_update(stored_rolling_window_dict):
    for _ in stored_rolling_window_dict:
        stored_rolling_window_dict[_].append(0)
        stored_rolling_window_dict[_].popleft()
    return stored_rolling_window_dict


def create_motor_rolling_window_len(length_window=0, current_rolling_window_dict={}, motor_id='0'):
    rolling_window_len = length_window
    rolling_window = current_rolling_window_dict.copy()
    motor_id = int(motor_id)
    if motor_id in rolling_window:
        rolling_window[motor_id].update([0] * (rolling_window_len))
    else:
        rolling_window[motor_id] = deque([0] * rolling_window_len)
    return rolling_window


def update_moving_average(moving_average, device_id, device_power):
    moving_average[device_id].append(device_power)
    moving_average[device_id].popleft()
    return moving_average
# Motor section ends


# Servo OPU starts
def servo_generate_power(power, feagi_power):
    return power * feagi_power


def update_servo_status_by_default(device_id, initialized_position):
    global servo_status
    if device_id not in servo_status:
        servo_status[device_id] = initialized_position
    print(f"device, {device_id}, id is initalized at: ", initialized_position)


def servo_negative_or_positive(id, power):
    if id % 2 == 0:
        power = abs(power)
    else:
        power = -1 * power
    return power


def update_power_of_servo(servo_from_servo_data, use_previous_servo_data=False):
    global servo_status, capabilities, actuators_mapped
    send_servo_data_to_controller = dict()
    for feagi_id in servo_from_servo_data:
        device_id_list = feagi_mapped_to_dev_index(dev_id='servo', feagi_index=feagi_id, mapped_dict=actuators_mapped)
        for device_id in device_id_list:
            if not capabilities['output']['servo'][str(device_id)]['disabled']:
                servo_power = servo_generate_power(capabilities['output']["servo"][str(device_id)]["max_power"],
                                                   servo_from_servo_data[feagi_id])
                pre_power = servo_status[device_id] + servo_power
                new_power = servo_keep_boundaries(pre_power,
                                                  capabilities['output']['servo'][str(device_id)]['max_value'],
                                                  capabilities['output']['servo'][str(device_id)]['min_value'])
                if use_previous_servo_data:
                    if device_id not in previous_servo_data or previous_servo_data[device_id] != new_power:
                        send_servo_data_to_controller[device_id] = new_power
                    servo_status[device_id] = new_power
                    previous_servo_data[device_id] = new_power
                else:
                    send_servo_data_to_controller[device_id] = new_power
                    servo_status[device_id] = new_power
    return send_servo_data_to_controller


def convert_feagi_to_servo_opu(obtained_data):
    servo_from_feagi_data = dict()
    if 'servo' in obtained_data:
        for data_point in obtained_data['servo']:
            device_power = servo_negative_or_positive(data_point, obtained_data['servo'][data_point])
            device_id = feagi_id_converter(data_point)
            if device_id in servo_from_feagi_data:
                servo_from_feagi_data[device_id] += device_power
            else:
                servo_from_feagi_data[device_id] = device_power
    return servo_from_feagi_data


def get_servo_data(obtained_data, use_previous_servo_data=False):
    global capabilities, servo_status
    converted_data = convert_feagi_to_servo_opu(obtained_data)
    return update_power_of_servo(converted_data, use_previous_servo_data)
# Servo OPU ends

# Servo Position starts
def get_servo_position_data(feagi_data, use_previous_servo_data=False):
    converted_data = convert_feagi_to_servo_position_opu(obtained_data=feagi_data)
    return update_power_of_servo_position(converted_data, use_previous_servo_data)


def update_power_of_servo_position(servo_from_servo_data, use_previous_servo_data=False):
    global servo_status, capabilities, actuators_mapped
    send_servo_data_to_controller = dict()
    for feagi_id in servo_from_servo_data:
        device_id_list = feagi_mapped_to_dev_index(dev_id='servo', feagi_index=feagi_id, mapped_dict=actuators_mapped)
        for device_id in device_id_list:
            if not capabilities['output']['servo'][str(device_id)]['disabled']:
                new_power = get_position_data(servo_from_servo_data[feagi_id],
                                                        capabilities['output']['servo'][str(device_id)]['min_value'],
                                                        capabilities['output']['servo'][str(device_id)]['max_value'])
                if use_previous_servo_data:
                    if device_id not in previous_servo_data or previous_servo_data[device_id] != new_power:
                        send_servo_data_to_controller[device_id] = new_power
                    servo_status[device_id] = new_power
                    previous_servo_data[device_id] = new_power
                else:
                    send_servo_data_to_controller[device_id] = new_power
                    servo_status[device_id] = new_power
    return send_servo_data_to_controller

def convert_feagi_to_servo_position_opu(obtained_data):
    servo_position_data = dict()
    if 'servo_position' in obtained_data:
        for data_point in obtained_data['servo_position']:
            device_power = obtained_data['servo_position'][data_point]
            device_id = data_point
            if device_id in servo_position_data:
                servo_position_data[device_id] += device_power
            else:
                servo_position_data[device_id] = device_power
    return servo_position_data


def check_emergency_stop(obtained_data):
    emergency_data = dict()  # I don't think there's any required input on emergency stop at all
    if 'emergency' in obtained_data:
        for data_point in obtained_data['emergency']:
            device_id = data_point
            device_power = obtained_data['emergency'][data_point]
            emergency_data[device_id] = device_power
    return emergency_data

def check_new_speed(obtained_data):
    speed_data = dict()  # I don't think there's any required input on emergency stop at all
    if 'speed' in obtained_data:
        for data_point in obtained_data['speed']:
            device_id = data_point
            device_power = obtained_data['speed'][data_point]
            speed_data[device_id] = device_power
    return speed_data

def check_actuator_in_capabilities(actuator):
    global capabilities
    try:
        if actuator in capabilities['output']:
            return True
        else:
            return False
    except:
        return False


def get_motion_control_data(obtained_data):
    global actuators_mapped, capabilities
    motion_control_data = dict()
    if check_actuator_in_capabilities('motion_control'):
        motion_control_data['motion_control'] = dict()
        if 'motion_control' in obtained_data:
            for data_point in obtained_data['motion_control']:
                device_id_list = feagi_mapped_to_dev_index(dev_id='motion_control', feagi_index=data_point, mapped_dict=actuators_mapped)
                for device_id in device_id_list:
                    device_power = obtained_data['motion_control'][data_point]
                    motion_control_data['motion_control'][device_id] = device_power
    return motion_control_data


def get_generic_opu_data_from_feagi(obtained_data, actuator_name):
    global actuators_mapped
    generic_data = dict()
    if actuator_name in obtained_data:
        if check_actuator_in_capabilities(actuator_name):
            for data_point in obtained_data[actuator_name]:
                device_id_list = feagi_mapped_to_dev_index(dev_id=actuator_name, feagi_index=data_point,
                                                           mapped_dict=actuators_mapped)
                for device_id in device_id_list:
                    device_id = device_id
                    device_power = obtained_data[actuator_name][data_point]
                    generic_data[device_id] = device_power
    return generic_data


def get_led_data(obtained_data):
    led_data = dict()
    if 'led' in obtained_data:
        for data_point in obtained_data['led']:
            led_data[data_point] = obtained_data['led'][data_point]
    return led_data


def servo_keep_boundaries(current_position, max=180, min=0):
    """
    Prevent Servo position to go beyond range
    """
    if current_position > max:
        adjusted_position = float(max)
    elif current_position < min:
        adjusted_position = float(min)
    else:
        adjusted_position = float(current_position)
    return adjusted_position


def get_gpio_data(obtained_data):
    gpio_data = dict()
    if 'gpio' in obtained_data:
        for data_point in obtained_data['gpio']:
            gpio_data[data_point] = obtained_data['gpio'][data_point]
    return gpio_data


def check_convert_gpio_to_input(obtained_data):
    input_gpio_data = dict()
    if 'gpio_input' in obtained_data:
        for data_point in obtained_data['gpio_input']:
            input_gpio_data[data_point] = obtained_data['gpio_input'][data_point]
    return input_gpio_data


def get_position_data(power, min_output, max_output):
    return power * (max_output - min_output) + min_output


def actuator_to_feagi_map(capabilities):
    """
    This function enables you to easily map between FEAGI and physical motors without requiring any manual edits.

    After using this function, you should be able to directly retrieve data from FEAGI and pass it to your function
    to control the robot's movement.

    The 'capabilities' parameter should follow the template below:
        "output": {
            "motor": {
                "0": {
                    "custom_name": "Motor_0",
                    "disabled": false,
                    "max_power": 4094,
                    "rolling_window_len": 2,
                    "feagi_index": 0
                },
                "1": {
                    "custom_name": "Motor_1",
                    "disabled": false,
                    "max_power": 4094,
                    "rolling_window_len": 2,
                    "feagi_index": 3
                },
    """
    feagi_to_actuator = {}
    for actuator_name in capabilities['output']:
        feagi_to_actuator[actuator_name] = {}
        for dev_index in capabilities['output'][actuator_name]:
            feagi_index = capabilities['output'][actuator_name][dev_index]['feagi_index']
            dev_index = int(dev_index)
            if feagi_index not in feagi_to_actuator[actuator_name]:
                feagi_to_actuator[actuator_name][feagi_index] = []
            feagi_to_actuator[actuator_name][feagi_index].append(dev_index)
    return feagi_to_actuator


def feagi_mapped_to_dev_index(dev_id, feagi_index, mapped_dict):
    if feagi_index not in mapped_dict[dev_id]:
        return []
    return mapped_dict[dev_id][feagi_index]
