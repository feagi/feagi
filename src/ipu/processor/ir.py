<<<<<<< HEAD
import sys
sys.path.insert(1, '../third_party/freenove/smart_car/')
import controller
=======
>>>>>>> feature-environment-handler
from inf import runtime_data
if runtime_data.hardware == 'raspberry_pi':
    import sys
    sys.path.insert(1, '../third_party/freenove/smart_car/')
    import controller


def convert_ir_to_fire_list():
    ir_controller = controller.IR()
    active_sensors = ir_controller.read()
    if active_sensors is not None:
        fire_list = list()
        for sensor_idx in active_sensors:
            for key in runtime_data.brain['ir_ipu']:
                if sensor_idx == runtime_data.brain['ir_ipu'][key]['soma_location'][0][0]:
                    fire_list.append(key)
        print(">>>>>>>>>>>>>>>> IR FIRE LIST: ", fire_list)
        runtime_data.fcl_queue.put({'ir_ipu': fire_list})
