import sys
import traceback
from inf import runtime_data


def motor_operator(motor_data):
    """
    motor_data = {
        motor_id: {
            "direction": 'F',
            "...": ...
        },
        2: {},
        3: {}
    }
    """
    try:
        # todo: Generalize the following section. using specifics for test only
        if motor_data:
            message_to_router = {"motor": motor_data}
            runtime_data.opu_pub.send(message=message_to_router)
            if runtime_data.parameters["Logs"]["print_burst_info"]:
                for motor_id in motor_data:
                    print(f">>>>>>>>>>>>>>>>>>>>>>>> motor {motor_id} "
                        f"activation command sent to router with direction {motor_data[motor_id]}")

    except Exception as e:
        print("ERROR at Motor module:", e)
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)
