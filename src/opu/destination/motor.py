import sys
import traceback
from inf import runtime_data
from inf.messenger import Pub
from inf import runtime_data
from importlib.machinery import SourceFileLoader


def motor_operator(motor_id, speed, power=None):
    # controller = SourceFileLoader("controller.py", runtime_data.hw_controller_path).load_module()

    import time
    # print("Operating a motor on %s %s" % (hw_model, hw_brand))

    message = dict()
    message['motor'] = {}
    message['motor'][motor_id] = speed

    try:
        # todo: Generalize the following section. using specifics for test only
        runtime_data.opu_pub.send(message=message)
        print(f">>>>>>>>>>>>>>>>>>>>>>>> {motor_id} activation command sent to router with speed {speed}")

    except Exception as e:
        # print("ERROR: Requested controller not available for %s %s" % (hw_model, hw_brand))
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)
