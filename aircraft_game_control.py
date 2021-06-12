import pyvjoy
import math
import time
import socket
import struct
import math

THR_RATE = 0.7
UDP_IP = "127.0.0.1"
UDP_PORT = 4242


import socket
import re
import numpy as np

DCS_UDP_IP = "127.0.0.1"
DCS_UDP_PORT = 27015


def toHexCmd(cmd):
    #convert -1 to 1 to 0 to 32768
    cmd = math.floor(32768 * (cmd +1) /2)
    if cmd < 0:
        return 0
    if cmd > 32768:
        return 32768
    return cmd

def pose_to_udp_msg(eul, T):
    return struct.pack("<dddddd", T[0], T[1], T[2], eul[0], eul[1], eul[2])

def JoyEXP(v,exp):
    if math.fabs(v) < 0.001:
        return 0
    return math.pow(math.fabs(v),exp)* v / math.fabs(v)

class GameTracker:
    def __init__(self, ip="127.0.0.1", port=4242):
        self.UDP_IP = ip
        self.UDP_PORT = 4242
        self.sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

        self.T0 = None
    
    def send_pose(self, eul, T):
        data = pose_to_udp_msg(eul, T)
        self.sock.sendto(data, (self.UDP_IP, self.UDP_PORT))


class VJoyManager(object):
    def __init__(self):
        self.j = pyvjoy.VJoyDevice(1)
    
    def set_joystick_x(self, value):
        self.j.set_axis(pyvjoy.HID_USAGE_X, toHexCmd(value))
    
    def set_joystick_y(self, value):
        self.j.set_axis(pyvjoy.HID_USAGE_Y, toHexCmd(value))
    
    def set_joystick_z(self, value):
        self.j.set_axis(pyvjoy.HID_USAGE_Z, toHexCmd(value))

    def set_joystick_rz(self, value):
        self.j.set_axis(pyvjoy.HID_USAGE_RZ, toHexCmd(value))

    def set_joystick_rx(self, value):
        self.j.set_axis(pyvjoy.HID_USAGE_RX, toHexCmd(value))

    def set_joystick_ry(self, value):
        self.j.set_axis(pyvjoy.HID_USAGE_RY, toHexCmd(value))

class DCSTelem():
    def __init__(self):
        print("Trying to connect to DCS Telem")
        self.dcs_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        self.dcs_sock.bind((DCS_UDP_IP, DCS_UDP_PORT))

        self.OK = False

    def update(self):
        msg, addr = self.dcs_sock.recvfrom(1024) # buffer size is 1024 bytes
        self.data = self.parse_data(msg)
        self.OK = True

    def parse_data(self, data):
        data = data.decode("utf-8") 
        m = re.findall(r'([a-zA-Z]+)=(-?\d+(\.\d*)?)', data)
        values = {}
        for k, v, _ in m:
            v = float(v)
            values[k] = v
        return values

class game_aircraft_control():
    def __init__(self):
        self.vjoyman = VJoyManager()
        self.tracker = GameTracker()
        self.telem = DCSTelem()
        
        self.ail = 0
        self.ele = 0
        self.rud = 0
        self.thr = 0.7
        self.view_yaw = 0
        self.view_pitch = 0
        self.screen_scale = 2000
        self.ailrate = 1.5
        self.elerate = 2.0
        
        self.view_rate = 70.0
        self.att_sp_rate = 2.0
        # self.control_style = "battlefield" 
        self.control_style = "warthunder" 

        #Param for F18
        self.p_roll = 3.0
        self.p_pitch = 5.0
        self.p_rollrate = 0.4
        self.p_pitchrate = 0.4

        self.p_yaw = 0.5

        self.yaw_sp = 0
        self.pitch_sp = 0
        self.roll_sp = 0

        #Note w means world frame
        self.yawrate_w_sp = 0
        
        self.yawrate_b_sp = 0
        self.pitchrate_b_sp = 0
        self.rollrate_b_sp = 0

    def get_ail(self):
        return self.ail

    def get_ele(self):
        return self.ele
    
    def get_rud(self):
        return self.rud
    
    def get_thr(self):
        return self.thr

    def set_mouse_aircraft_control(self, _x, _y):
        x_sp =  _x / self.screen_scale 
        y_sp =  _y / self.screen_scale 
        if self.control_style == "battlefield":
            self.ail = x_sp*self.ailrate
            self.ele = -y_sp*self.elerate
        elif self.control_style == "warthunder":
            self.roll_sp = x_sp*self.att_sp_rate
            # self.yaw_sp = x_sp*self.att_sp_rate
            self.pitch_sp = -y_sp*self.att_sp_rate

    
    def control_body_aim(self, yaw_sp, pitch_sp):
        #need to update to quaternion
        dyaw = self.yaw_sp - self.telem.data["yaw"]
        phases = (dyaw + np.pi) % (2 * np.pi) - np.pi

    def controller_update(self):
        if self.telem.OK:

            # self.roll_sp = dyaw*self.p_yaw
            self.ail = (self.roll_sp - self.telem.data["roll"])*self.p_roll - self.p_rollrate*self.telem.data["rollrate"]
            # self.ail = self.roll_sp - self.p_rollrate*self.telem.data["rollrate"]
            self.ele = (self.pitch_sp - self.telem.data["pitch"])*self.p_pitch - self.p_pitchrate*self.telem.data["pitchrate"]
            print(f"yawsp {self.yaw_sp*57.3:3.1f} rollsp {self.roll_sp*57.3:3.1f} roll {self.telem.data['roll']*57.3:3.1f} rollrate {self.telem.data['rollrate']*57.3:3.1f}  ail {self.ail}")
            # print(f"rollsp {self.roll_sp*57.3:3.1f} roll {self.telem.data['roll']*57.3:3.1f} rollrate {self.telem.data['rollrate']*57.3:3.1f}  ail {self.ail}")
            # print(f"pitch_sp {self.pitch_sp} pitch {self.telem.data['pitch']} ele {self.ele}")

    def set_mouse_free_look(self, _x, _y):
        self.view_yaw = _x/ self.screen_scale*57.3*self.view_rate
        self.view_pitch = -_y/ self.screen_scale*57.3*self.view_rate

    def inc_thr(self, dt):
        self.thr += dt*THR_RATE
        if self.thr > 1:
            self.thr = 1

    def dec_thr(self, dt):
        self.thr -= dt*THR_RATE
        if self.thr < 0:
            self.thr = 0

    def update(self):
        self.telem.update()
        self.controller_update()
        self.vjoyman.set_joystick_x(self.ail)
        self.vjoyman.set_joystick_y(self.ele)
        self.vjoyman.set_joystick_rz(self.rud)
        self.vjoyman.set_joystick_z(-self.thr)
        self.tracker.send_pose([self.view_yaw, self.view_pitch, 0], [0, 0, 0])

if __name__ == '__main__':
    aircraft_con = game_aircraft_control()

    t = 0
    while True:
        aircraft_con.update()
        t+= 0.01
        time.sleep(0.01)