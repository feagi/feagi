from enum import Enum

class FEAGIConnectionStatus(Enum):
    DISCONNECTED = 0
    UNABLE_TO_CONNECT = -1
    UNABLE_TO_AUTHENTICATE = -2
    FEAGI_NOT_READY = -3
    CONNECTING = 1
    AUTHENTICATING = 2
    FEAGI_READY = 3