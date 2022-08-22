import requests
import controller
import configuration
import router
import time
import math
import RPi.GPIO as GPIO
from datetime import datetime


def main(args=None):
    GPIO.cleanup()
    print("Connecting to FEAGI resources...")

    # address = 'tcp://' + network_settings['feagi_host'] + ':' + network_settings['feagi_outbound_port']

    feagi_host = configuration.network_settings["feagi_host"]
    api_port = configuration.network_settings["feagi_api_port"]

    controller.feagi_registration(feagi_host=feagi_host, api_port=api_port)

    print("** **", controller.runtime_data["feagi_state"])
    configuration.network_settings['feagi_burst_speed'] = float(controller.runtime_data["feagi_state"]['burst_duration'])

    # todo: to obtain this info directly from FEAGI as part of registration
    ipu_channel_address = 'tcp://0.0.0.0:' + controller.runtime_data["feagi_state"]['feagi_inbound_port_gazebo']
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = 'tcp://' + configuration.network_settings['feagi_host'] + ':' + \
                          controller.runtime_data["feagi_state"]['feagi_outbound_port']

    feagi_ipu_channel = router.Pub(address=ipu_channel_address)
    feagi_opu_channel = router.Sub(address=opu_channel_address, flags=router.zmq.NOBLOCK)

    flag = False
    counter = 0
    old_opu_data = {}
    print("Connecting to FEAGI resources...")

    feagi_host = configuration.network_settings["feagi_host"]
    api_port = configuration.network_settings["feagi_api_port"]
    api_address = 'http://' + feagi_host + ':' + api_port
    stimulation_period_endpoint = '/v1/feagi/feagi/burst_engine/stimulation_period'
    burst_counter_endpoint = '/v1/feagi/feagi/burst_engine/burst_counter'

    motor = controller.Motor()
    ir = controller.IR()
    ultrasonic = controller.Ultrasonic()
    # battery = controller.Battery()
    servo = controller.Servo()

    rolling_window_len = configuration.capabilities['motor']['rolling_window_len']
    motor_count = configuration.capabilities['motor']['count']
    msg_counter = 0
    # LED.test_Led()

    rolling_window = {}
    for motor_id in range(motor_count):
        rolling_window[motor_id] = controller.deque([0] * rolling_window_len)

    RPM = (50 * 60) / 2  # DC motor has 2 poles, 50 is the freq and it's constant (why??) and 60 is the seconds of a minute
    w = (RPM / 60) * (2 * math.pi)  # 60 is second/minute
    velocity = w * (configuration.capabilities['motor']['diameter_of_wheel'] / 2)
    #^ diameter is from config and it just needs radius so I turned the diameter into a radius by divide it with 2

    try:
        while True:
            # transmit data to FEAGI IPU
            ir_data = ir.read()
            if ir_data:
                formatted_ir_data = {'ir': {sensor: True for sensor in ir_data}}
            else:
                formatted_ir_data = {}

            if ir_data:
                for ir_sensor in range(int(configuration.capabilities['infrared']['count'])):
                    if ir_sensor not in formatted_ir_data['ir']:
                        formatted_ir_data['ir'][ir_sensor] = False
            else:
                formatted_ir_data['ir'] = {}
                for ir_sensor in range(int(configuration.capabilities['infrared']['count'])):
                    formatted_ir_data['ir'][ir_sensor] = False

            for ir_sensor in range(int(configuration.capabilities['infrared']['count'])):
                if ir_sensor not in formatted_ir_data['ir']:
                    formatted_ir_data['ir'][ir_sensor] = False

            ultrasonic_data = ultrasonic.get_distance()
            if ultrasonic_data:
                formatted_ultrasonic_data = {
                    'ultrasonic': {
                        sensor: data for sensor, data in enumerate([ultrasonic_data])
                    }
                }
            else:
                formatted_ultrasonic_data = {}

            controller.compose_message_to_feagi(
                original_message={**formatted_ir_data, **formatted_ultrasonic_data}) # Removed battery due to error
            # Process OPU data received from FEAGI and pass it along
            message_from_feagi = feagi_opu_channel.receive()
           # print("Received:", opu_data)
            if message_from_feagi is not None:
                opu_data = message_from_feagi["opu_data"]

                if 'o__mot' in opu_data:
                    for data_point in opu_data['o__mot']:
                        data_point = controller.block_to_array(data_point)
                        device_id = motor.motor_converter(data_point[0])
                        device_power = data_point[2]
                        device_power = motor.power_convert(data_point[0], device_power)
                        # RPM = (50 * 60) / 2 # DC motor has 2 poles, 50 is the freq and it's constant (why??) and 60 is the seconds of a minute
                        # w = (RPM / 60) * (2 * math.pi)  #60 is second/minute
                        # velocity = w * (capabilities['motor']['diameter_of_wheel']/2) # diameter is from config and it just needs radius so I turned the diameter into a radius by divide it with 2
                        motor.move(device_id, (device_power * 455))
                        # flag = True
                    # if opu_data['o__mot']  == {}:
                    #     motor.stop()  # When it's empty inside opu_data['o__mot']
                #for data_point in opu_data['o__mot'].keys():
                    #print("",data_point)
                    #if not data_point in old_opu_data:
                        #print("key is missing: ", data_point)
                        #print("datapoint: ",data_point[data_point])
                        #print(opu_data['o__mot'])
                        #print(type(opu_data['o__mot']))
                        #print(type(data_point))
                        #print(data_point[0])
                        #print(data_point[2])
                        # device_id = motor.motor_converter(int(data_point[0]))
                        # device_power = data_point[2]
                        # device_power = motor.power_convert(data_point[0], device_power)
                        # motor.move(device_id, 0)
                #old_opu_data['o__mot'] = opu_data['o__mot'].copy()
                if 'o__ser' in opu_data:
                    for data_point in opu_data['o__ser']:
                        data_point = controller.block_to_array(data_point)
                        device_id = data_point[0]
                        device_power = data_point[2]
                        # device_id = servo.servo_id_converter(device_id)
                        # device_power = servo.power_convert(data_point[0], device_power)
                        servo.move(feagi_device_id=device_id, power=device_power)
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
                    feagi_opu_channel = router.Sub(address=opu_channel_address, flags=router.zmq.NOBLOCK)
                    if feagi_burst_speed != configuration.network_settings['feagi_burst_speed']:
                        configuration.network_settings['feagi_burst_speed'] = feagi_burst_speed
            time.sleep((configuration.network_settings['feagi_burst_speed'])/velocity)
            motor.stop()
            # if flag:
            #     if counter < 3:
            #         counter += msg_counter
            #     else:
            #         motor.stop()
            #         counter = 0

            # LED.leds_off()
    except KeyboardInterrupt as ke: ##Keyboard error
        motor.stop()
        print(ke)


if __name__ == '__main__':
    main()