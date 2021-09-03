import sys
sys.path.insert(1, '../third_party/freenove/smart_car/')
import controller
from inf import runtime_data


def convert_ir_to_fire_list():
    ir_controller = controller.IR()
    active_sensors = ir_controller.read()
    if active_sensors is not None:
        fire_list = list()
        for sensor_idx in active_sensors:
            for key in runtime_data.brain['infrared_sensor']:
                if sensor_idx == runtime_data.brain['infrared_sensor'][key]['soma_location'][0][0]:
                    fire_list.append(key)
        runtime_data.fcl_queue.put({'infrared_sensor': fire_list})
