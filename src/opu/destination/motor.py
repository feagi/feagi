import sys
import traceback
from inf import runtime_data


def motor_operator(motor_data):
    """
    motor_data = {
        motor_id: {
            "speed": 3,
            "power": 2,
            "...": ...
        },
        2: {},
        3: {}
    }
    """
    try:
        # todo: Generalize the following section. using specifics for test only
        runtime_data.opu_pub.send(message=motor_data)
        for motor_id in motor_data:
            print(f">>>>>>>>>>>>>>>>>>>>>>>> {motor_id} "
                  f"activation command sent to router with speed {motor_data[motor_id]['speed']}")

    except Exception as e:
        print("ERROR at Motor module:", e)
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)
