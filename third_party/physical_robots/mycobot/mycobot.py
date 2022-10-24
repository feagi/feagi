from pymycobot.mycobot import MyCobot
from feagi_agent import feagi_interface as FEAGI
from feagi_agent import retina as retina
from configuration import *
import time


class Arm:
    @staticmethod
    def connection_initialize(port='/dev/ttyUSB0'):
        """
        :param port: The default would be '/dev/ttyUSB0'. If the port is different, put a different port.
        :return:
        """
        return MyCobot(port)

    @staticmethod
    def get_coordination(robot):
        """
        :return: 6 servos' coordination
        """
        data = robot.get_coords()
        return data

    def move(self, robot, encoder_id, power):
        """
        :param encoder_id: servo ID
        :param power: A power to move to the point
        """
        if encoder_id not in runtime_data['servo_status']:
            runtime_data['servo_status'][encoder_id] = power
        print("encoder_id: ", encoder_id)
        print("power: ", runtime_data['servo_status'][encoder_id])
        if capabilities['servo']['servo_range'][1] >= (runtime_data['servo_status'][encoder_id] + power) >= \
                capabilities['servo']['servo_range'][0]:
            robot.set_encoder(encoder_id, runtime_data['servo_status'][encoder_id])
            runtime_data['servo_status'][encoder_id] += power

    @staticmethod
    def power_convert(encoder_id, power):
        if encoder_id % 2 == 0:
            return -1 * power
        else:
            return abs(power)

    @staticmethod
    def encoder_converter(encoder_id):
        """
        This will convert from godot to motor's id. Let's say, you have 8x10 (width x depth from static_genome).
        So, you click 4 to go forward. It will be like this:
        o__mot': {'1-0-9': 1, '5-0-9': 1, '3-0-9': 1, '7-0-9': 1}
        which is 1,3,5,7. So this code will convert from 1,3,5,7 to 0,1,2,3 on motor id.
        Since 0-1 is motor 1, 2-3 is motor 2 and so on. In this case, 0 is for forward and 1 is for backward.
        """
        if encoder_id <= 1:
            return 1
        elif encoder_id <= 3:
            return 2
        elif encoder_id <= 5:
            return 3
        elif encoder_id <= 7:
            return 4
        elif encoder_id <= 9:
            return 5
        elif encoder_id <= 11:
            return 6
        else:
            print("Input has been refused. Please put encoder ID.")

runtime_data = {
    "current_burst_id": 0,
    "feagi_state": None,
    "cortical_list": (),
    "battery_charge_level": 1,
    "host_network": {},
    'motor_status': {},
    'servo_status': {}
}

# # # FEAGI registration # # #
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


mycobot = Arm()
arm = mycobot.connection_initialize()
print(mycobot.get_coordination(arm))
print(arm.release_servo(1))
print("IS CONTROLLER CONNECTED?: ", arm.is_controller_connected())
print("is all servo enabled", arm.is_all_servo_enable())

# for i in range(1,6,1):
#     arm.set_servo_calibration(i)
# while True:
#     print(str(1) + "'S STATUS: ", arm.get_encoder(2))
#     print(str(2) + " ENABLED?: ", arm.is_servo_enable(2))
#     # print(str(3) + "'S STATUS: ", arm.get_encoder(3))
#     # print(str(4) + "'S STATUS: ", arm.get_encoder(4))
print("version: ", arm.get_system_version())
print("DONE!")
flag = True
while flag:
    try:
        message_from_feagi = feagi_opu_channel.receive()
        if message_from_feagi is not None:
            opu_data = FEAGI.opu_processor(message_from_feagi)
            if 'motor' in opu_data:
                if opu_data['motor'] is not {}:
                    for data_point in opu_data['motor']:
                        device_power = opu_data['motor'][data_point]
                        device_power = mycobot.power_convert(data_point, device_power)
                        device_id = mycobot.encoder_converter(data_point)
                        mycobot.move(arm, device_id, device_power)
    except KeyboardInterrupt as ke:  # Keyboard error
        arm.release_all_servos()
        flag = False

