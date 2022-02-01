
"""
This module contains a virtual set of hardware controllers used for simulation and testing only

todo: Need to have all of the controller.py modules follow the same convention and be consistent especially input data
"""
import router
import configuration
from time import time, sleep
from fake_stimulation import stimulation_data
from random import randrange, getrandbits


runtime_params = {
    "current_burst_id": 0,
    "global_timer": 0.5,
    "feagi_state": None,
    "cortical_list": ()
}


class FakeStimulator:
    def __init__(self):
        print("Fake stimulation initialized")

    @staticmethod
    def stimulate(burst_id):
        stimuli = dict()
        if burst_id in stimulation_data:
            print("Cortical_list: ", runtime_params["cortical_list"])
            for cortical_area in stimulation_data[burst_id]:
                if cortical_area in runtime_params["cortical_list"]:
                    stimuli[cortical_area] = stimulation_data[burst_id][cortical_area]
            return stimuli


def build_message_to_feagi():
    """
    This function encodes the sensory information in a dictionary that can be decoded on the FEAGI end.

    expected ipu_data structure:

        ipu_data = {
            "capabilities": {},
            "network": {},
            "data": {
                "direct_stimulation": {
                    "cortical_area_id": {voxel},
                    "cortical_area_id": sensor_data,
                    "cortical_area_id": sensor_data
                    ...
                    },
                "sensory_data": {
                    "sensor type": sensor data,
                    "sensor type": sensor data,
                    "sensor type": sensor data,
                    ...
                }
            }
    """

    stimulator = FakeStimulator()
    # Process IPU data received from controller.py and pass it along to FEAGI
    # todo: move class instantiations to outside function
    # ir = IR()

    # todo: figure a better way of obtaining the device count
    # ir_count = 3

    message = dict()
    message["controller_burst_id"] = runtime_params["current_burst_id"]
    message['data'] = dict()
    message['data']["direct_stimulation"] = dict()
    message['data']["sensory_data"] = dict()
    message['data']["direct_stimulation"] = stimulator.stimulate(runtime_params["current_burst_id"])
    if runtime_params["current_burst_id"] % 10 == 0:
        message["capabilities"] = configuration.capabilities
        message["network"] = configuration.network_settings

    # ipu_data['ultrasonic'] = {
    #     1: [randrange(0, 30) / 10, randrange(0, 30) / 10, randrange(0, 30) / 10, randrange(0, 30) / 10,
    #         randrange(0, 30) / 10, randrange(0, 30) / 10]
    # }
    # ipu_data['ir'] = {}

    # for _ in range(ir_count):
    #     ipu_data['ir'][_] = ir.read()

    return message


def cortical_mapping_list_gen(capabilities):
    cortical_list = set()
    for device in capabilities:
        cortical_list.add(capabilities[device]["cortical_mapping"])
    return cortical_list


def main():
    runtime_params["cortical_list"] = cortical_mapping_list_gen(configuration.capabilities)

    address = 'tcp://' + configuration.network_settings['feagi_ip'] + ':' + \
              configuration.network_settings['feagi_outbound_port']
    
    runtime_params["feagi_state"] = router.handshake_with_feagi(address=address, capabilities=configuration.capabilities)

    print("** **", runtime_params["feagi_state"])

    # todo: to obtain this info directly from FEAGI as part of registration
    ipu_channel_address = 'tcp://0.0.0.0:' + configuration.network_settings['feagi_inbound_port']
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = 'tcp://' + configuration.network_settings['feagi_ip'] + ':' + \
                          runtime_params["feagi_state"]['sockets']['feagi_outbound_port']

    feagi_ipu_channel = router.Pub(address=ipu_channel_address)
    feagi_opu_channel = router.Sub(address=opu_channel_address, flags=router.zmq.NOBLOCK)

    print("Connecting to FEAGI resources...")

    # todo: identify a method to instantiate all classes without doing it one by one
    # Instantiate Controller Classes
    # motor = Motor()

    # Listen and route
    print("Starting the routing engine")
    print("Communication frequency is set once every %f seconds" % runtime_params['global_timer'])

    # todo: need to have a method to sync burst id with the FEAGI
    runtime_params["current_burst_id"] = runtime_params["feagi_state"]["burst_counter"]

    while True:
        # Process OPU data received from FEAGI and pass it along to the controller.py
        opu_data = feagi_opu_channel.receive()
        print("Received:", opu_data)
        # if opu_data is not None:
        #     if 'motor' in opu_data:
        #         for motor_id in opu_data['motor']:
        #             motor.move(motor_id, opu_data['motor'][motor_id])
        #

        message_to_feagi = build_message_to_feagi()
        feagi_ipu_channel.send(message_to_feagi)

        # todo: IMPORTANT!!! need to figure how to correlate the flow on incoming data with the rate data is passed to FEAGI
        sleep(runtime_params['global_timer'])
        if opu_data:
            runtime_params["current_burst_id"] = opu_data['burst_counter']
            if opu_data['burst_counter'] == 5:
                print("5!!!!")


if __name__ == '__main__':
    main()




# class Motor:
#     def __init__(self):
#         print("Neuraville virtual motor has been initialized...")
#
#     def move(self, motor_index, speed):
#         print("<< < .$.Speed of motor %s is set to %s.$. > >>" % (motor_index, speed))

#
# class Photoresistor:
#     """
#     Photoresistor has two photoresistors. 0 is the left photoresistor and 1 is the right photoresistor on the board.
#     """
#     def photoresistor(self, number):  # 0 is the left photoressitor and 1 is the right photoresistor
#         adc = Adc()
#         if number > 2 or number < 0:
#             print("Please put 1 or 0 as an input only")
#         elif number < 2 or number <= 0:
#             output = adc.recvADC(number)
#             # print(output)
#             return output
#
#
# class Battery:
#     def battery_total(self):  ##It gives a full volt remain of battery
#         """
#         Returns a fake battery value
#         -------
#
#         """
#         power = 2.3
#         return power
#
#
# class Ultrasonic:
#     def __init__(self):
#         GPIO.setwarnings(False)
#         self.trigger_pin = 27
#         self.echo_pin = 22
#         GPIO.setmode(GPIO.BCM)
#         GPIO.setup(self.trigger_pin,GPIO.OUT)
#         GPIO.setup(self.echo_pin,GPIO.IN)
#
#     def send_trigger_pulse(self):
#         GPIO.output(self.trigger_pin,True)
#         time.sleep(0.00015)
#         GPIO.output(self.trigger_pin,False)
#
#     def wait_for_echo(self,value,timeout):
#         count = timeout
#         while GPIO.input(self.echo_pin) != value and count>0:
#             count = count-1
#
#     def getDistance(self):
#         """
#
#         Returns distance_cm in INT value
#         -------
#
#         """
#         distance_cm=[0,0,0]
#         for i in range(3):
#             self.send_trigger_pulse()
#             self.wait_for_echo(True,10000)
#             start = time.time()
#             self.wait_for_echo(False,10000)
#             finish = time.time()
#             pulse_len = finish-start
#             distance_cm[i] = pulse_len/0.000058
#         distance_cm=sorted(distance_cm)
#         return int(distance_cm[1])
#
#
# class LED:
#     def __init__(self):
#         self.led = Led()
#
#     def LED_on(self, led_ID, Red_Intensity, Blue_Intensity, Green_intensity):
#         """
#         Parameters
#         ----------
#         led_ID: This is the ID of leds. It can be from 1 to 8
#         Red_Intensity: 1 to 255, from dimmest to brightest
#         Blue_Intensity: 1 to 255, from dimmest to brightest
#         Green_intensity: 1 to 255, from dimmest to brightest
#         -------
#         """
#         try:
#             self.led.ledIndex(led_ID, Red_Intensity, Blue_Intensity, Green_intensity)
#         except KeyboardInterrupt:
#             self.led.colorWipe(led.strip, Color(0, 0, 0))  ##This is to turn all leds off/
#
#     def test_Led(self):
#         """
#         This is to test all leds and do several different leds.
#         """
#         try:
#             self.led.ledIndex(0x01, 255, 0, 0)  # Red
#             self.led.ledIndex(0x02, 255, 125, 0)  # orange
#             self.led.ledIndex(0x04, 255, 255, 0)  # yellow
#             self.led.ledIndex(0x08, 0, 255, 0)  # green
#             self.led.ledIndex(0x10, 0, 255, 255)  # cyan-blue
#             self.led.ledIndex(0x20, 0, 0, 255)  # blue
#             self.led.ledIndex(0x40, 128, 0, 128)  # purple
#             self.led.ledIndex(0x80, 255, 255, 255)  # white'''
#             print("The LED has been lit, the color is red orange yellow green cyan-blue blue white")
#             time.sleep(3)  # wait 3s
#             self.led.colorWipe("", Color(0, 0, 0))  # turn off the light
#             print("\nEnd of program")
#         except KeyboardInterrupt:
#             self.led.colorWipe("", Color(0, 0, 0))  # turn off the light
#             print("\nEnd of program")
#
#     def leds_off(self):
#         self.led.colorWipe("", Color(0, 0, 0))  ##This is to turn all leds off/
#
#
# class IR:
#     def __init__(self):
#         print("Infrared module has ben activated...")
#
#     def read(self):
#         ir_reading = bool(getrandbits(1))
#         return ir_reading
#
#
# class Buzzer(object):
#     def __init__(self):
#         GPIO.setmode(GPIO.BCM)
#         self.buzzer_pin = 17  # set to GPIO pin 17. That's where the buzzer included.
#         GPIO.setup(self.buzzer_pin, GPIO.IN)
#         GPIO.setup(self.buzzer_pin, GPIO.OUT)
#         print("buzzer ready")
#
#     def __del__(self):
#         class_name = self.__class__.__name__
#         print(class_name, "finished")
#
#     def buzz(self, pitch, duration):  # create the function “buzz” and feed it the pitch and duration)
#         """
#         Parameters
#         ----------
#         pitch: pitch level
#         duration: Seconds
#         -------
#         """
#
#         if pitch == 0:
#             time.sleep(duration)
#             return
#         period = 1.0 / pitch  # in physics, the period (sec/cyc) is the inverse of the frequency (cyc/sec)
#         delay = period / 2  # calcuate the time for half of the wave
#         cycles = int(duration * pitch)  # the number of waves to produce is the duration times the frequency
#
#         for i in range(cycles):  # start a loop from 0 to the variable “cycles” calculated above
#             GPIO.output(self.buzzer_pin, True)  # set pin 18 to high
#             time.sleep(delay)  # wait with pin 18 high
#             GPIO.output(self.buzzer_pin, False)  # set pin 18 to low
#             time.sleep(delay)  # wait with pin 18 low
#
#     def play(self, pitch_level, seconds): # The higher number, the higher pitch. The lower number, the lower pitch.
#         """
#         Parameters
#         ----------
#         pitch_level: pitch level
#         seconds: duration
#         -------
#         """
#         GPIO.setmode(GPIO.BCM)
#         GPIO.setup(self.buzzer_pin, GPIO.OUT)
#         x = 0
#         pitches = [pitch_level]
#         duration = seconds
#         for p in pitches:
#             self.buzz(p, duration)  # feed the pitch and duration to the function, “buzz”
#             time.sleep(duration * 0.5)
#
#         GPIO.setup(self.buzzer_pin, GPIO.IN)
#
#
# class Servo:
#     """
#     Functions: head_UP_DOWN and head_RIGHT_LEFT only. Other functions are just a support and defined system for Servo
#     class to work with functions.
#     """
#     def __init__(self):
#         print("Neuraville virtual servo has been initialized...")
#
#     def setServoPwm(self,channel,angle,error=10):
#         angle = int(angle)
#         if channel == '0':
#             self.PwmServo.setServoPulse(8,2500-int((angle+error)/0.09))
#         elif channel == '1':
#             self.PwmServo.setServoPulse(9,500+int((angle+error)/0.09))
#         elif channel == '2':
#             self.PwmServo.setServoPulse(10,500+int((angle+error)/0.09))
#         elif channel == '3':
#             self.PwmServo.setServoPulse(11,500+int((angle+error)/0.09))
#         elif channel == '4':
#             self.PwmServo.setServoPulse(12,500+int((angle+error)/0.09))
#         elif channel == '5':
#             self.PwmServo.setServoPulse(13,500+int((angle+error)/0.09))
#         elif channel == '6':
#             self.PwmServo.setServoPulse(14,500+int((angle+error)/0.09))
#         elif channel == '7':
#             self.PwmServo.setServoPulse(15,500+int((angle+error)/0.09))
#
#     def head_UP_DOWN(self, num):
#         """
#         Parameters
#         ----------
#         num: degree from 0 to 180.
#         -------
#         """
#         self.setServoPwm('1', num) #90 to 0 degree is turn the head down. 90 to 180 is to turn the head up
#
#     def head_RIGHT_LEFT(self, num):
#         """
#         Parameters
#         ----------
#         num: degree from 0 to 180.
#         -------
#         """
#         self.setServoPwm('0', num) #90 to 0 degree is turn the head left. 90 to 180 is to turn the head right
