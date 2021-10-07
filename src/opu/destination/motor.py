import sys
import traceback
from inf import runtime_data


def motor_operator(motor_id, motor_output, power=None):
    message = dict()
    message['motor'] = {}
    message['motor'][motor_id] = motor_output

    try:
        # todo: Generalize the following section. using specifics for test only
        runtime_data.opu_pub.send(message=message)
        print(f">>>>>>>>>>>>>>>>>>>>>>>> {motor_id} activation command sent to router with speed {motor_output}")

    except Exception as e:
        print("ERROR at Motor module:", e)
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)
