
from inf import runtime_data


def convert_ir_to_fire_list(ir_data):
    """

    The keys in ir_data correlate to the index id of each Infrared Sensor

    ir_data = {
        0: True,
        1: True,
        2: False
    }
    """
    fire_list = set()
    for sensor_idx in ir_data:
        if ir_data[sensor_idx]:
            for key in runtime_data.brain['ir_ipu']:
                if sensor_idx == runtime_data.brain['ir_ipu'][key]['soma_location'][0][0]:
                    fire_list.add(key)
    temp = runtime_data.fcl_queue.get()
    temp['ir_ipu'].add(fire_list)
    runtime_data.fcl_queue.put({'ir_ipu': temp['ir_ipu']})
