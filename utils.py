import math
import numpy as np
import struct

def float_constrain(v, min, max):
    if v < min:
        v = min
    if v > max:
        v = max
    return v

def toHexCmd(cmd):
    #convert -1 to 1 to 0 to 32768
    cmd = math.floor(32768 * (cmd +1) /2)
    if cmd < 0:
        return 0
    if cmd > 32768:
        return 32768
    return cmd

def wrap_pi(v):
    return (v + np.pi) % (2 * np.pi) - np.pi

def pose_to_udp_msg(eul, T):
    return struct.pack("<dddddd", T[0], T[1], T[2], eul[0], eul[1], eul[2])

def JoyEXP(v,exp):
    if math.fabs(v) < 0.001:
        return 0
    return math.pow(math.fabs(v),exp)* v / math.fabs(v)