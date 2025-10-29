from enum import Enum

class AgentState(Enum):
    NotConnected = 0,
    Connecting = 1,
    Registering = 2,
    Running = 3
