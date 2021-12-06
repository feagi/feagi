import sys
import traceback
from inf import runtime_data


def servo_operator(servo_data):
    """
    servo_data = {
        servo_id: {
            "angle": 30
        },
        2: {},
        3: {}
    }
    """
    try:
        # todo: Generalize the following section. using specifics for test only
        message_to_router = {"servo": servo_data}
        runtime_data.opu_pub.send(message=message_to_router)
        if runtime_data.parameters["Logs"]["print_burst_info"]:
            for servo_id in servo_data:
                print(f">>>>>>>>>>>>>>>>>>>>>>>> {servo_id} "
                      f"activation command sent to router with angle {servo_data[servo_id]['angle']}")

    except Exception as e:
        print("ERROR at Servo module:", e)
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)
