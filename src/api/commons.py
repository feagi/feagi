from queue import Queue
from datetime import datetime

from ..inf import runtime_data


api_queue = Queue()


class CustomError(Exception):
    def __init__(self, message, status_code):
        self.message = message
        self.status_code = status_code
        print(message, status_code)


def pending_amalgamation():
    if not runtime_data.pending_amalgamation:
        return False
    else:
        # todo: externalize the amalgamation timeout currently hardcoded below
        amalgamation_reqeust_timeout = 500
        elapsed_time = datetime.now() - runtime_data.pending_amalgamation["initiation_time"]
        if elapsed_time.seconds > amalgamation_reqeust_timeout:
            print(f"Pending amalgamation got voided due to exceeding {amalgamation_reqeust_timeout} threshold! ")
            runtime_data.pending_amalgamation = {}
            return False
        else:
            return True


def cancel_pending_amalgamation(amalgamation_id):
    runtime_data.pending_amalgamation = {}
    if amalgamation_id in runtime_data.amalgamation_history:
        runtime_data.amalgamation_history[amalgamation_id] = "cancelled"
