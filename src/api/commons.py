from queue import Queue

api_queue = Queue()


class CustomError(Exception):
    def __init__(self, message, status_code):
        self.message = message
        self.status_code = status_code
        print(message, status_code)
