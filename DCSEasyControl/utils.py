import math
import numpy as np
import struct
from transformations import *

def float_constrain(v, min, max):
    if v < min:
        v = min
    if v > max:
        v = max
    return v

def setZeroRoll(q):
    q /= np.linalg.norm(q)
    r, p, y = euler_from_quaternion(q)
    r = 0
    return quaternion_from_euler(r, p, y)

def setZeroYaw(q):
    q /= np.linalg.norm(q)
    r, p, y = euler_from_quaternion(q)
    y = 0
    return quaternion_from_euler(r, p, y)

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

def att_err_to_tangent_space(q_sp, q):
    dq = quaternion_multiply(quaternion_inverse(q), q_sp)
    angle, axis, _= rotation_from_matrix(quaternion_matrix(dq))
    dw = angle*axis
    return dw

def quaternion_rotate(q, v):
    mat = quaternion_matrix(q)[0:3,0:3]
    v = np.array([v])
    v = v.transpose()
    v = mat @ v
    v = v.transpose()[0]
    return v

'''
void makeRotationDir(const Vec3& direction, const Vec3& up = Vec3(0,1,0))
    {
        Vec3 xaxis = Vec3::Cross(up, direction);
        xaxis.normalizeFast();

        Vec3 yaxis = Vec3::Cross(direction, xaxis);
        yaxis.normalizeFast();

        column1.x = xaxis.x;
        column1.y = yaxis.x;
        column1.z = direction.x;

        column2.x = xaxis.y;
        column2.y = yaxis.y;
        column2.z = direction.y;

        column3.x = xaxis.z;
        column3.y = yaxis.z;
        column3.z = direction.z;
    }
'''

def dir_to_q(direction, up=np.array([0, 0, 1])):
    direction = unit_vector(direction)
    yaxis = np.cross(up, direction)
    zaxis = np.cross(direction, yaxis)
    R = np.array([
        [direction[0], direction[1], direction[2], 0],
        [yaxis[0], yaxis[1], yaxis[2], 0],
        [zaxis[0], zaxis[1], zaxis[2], 0],
        [0, 0, 0, 1]
    ])
    R = R.transpose()
    return quaternion_from_matrix(R)

def q_to_dir(q):
    q = quaternion_rotate(q, np.array([1, 0, 0]))
    return q

if __name__ == "__main__":
    q = dir_to_q([0.4, 0.5, 0])
    print(q)
    dir = q_to_dir(q)
    print(dir)