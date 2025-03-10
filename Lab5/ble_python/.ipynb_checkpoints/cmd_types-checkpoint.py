from enum import Enum

class CMD(Enum):
    PING = 0
    GET_IMU_DATA = 1
    GET_TOF_DATA = 2
    START_COLLECTION = 3
    DRIVE = 4
    GET_PID = 5
