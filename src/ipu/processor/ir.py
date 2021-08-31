import controller
from inf import runtime_data


def convert_ir_to_fire_list():
    ir_controller = controller.IR()
    active_sensors = ir_controller.read()
    if active_sensors is not None:
        for sensor_idx in active_sensors:
            for key in runtime_data.brain["line_tracking"]:
                if sensor_idx == runtime_data.brain['line_tracking'][key]['soma_location'][0][0]:
                    runtime_data.fcl_queue.put({'line_tracking': key})