from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    SEND_THREE_FLOATS = 2
    ECHO = 3
    DANCE = 4
    SET_VEL = 5
    GET_TIME_MILLIS = 6
    START_TIME_LOOP = 7
    SEND_TIME_DATA = 8
    GET_TEMP_READINGS = 9
    INT_LOOP = 10
    INIT_VAR = 11
    VAR_BYTE_RESPONSE = 12