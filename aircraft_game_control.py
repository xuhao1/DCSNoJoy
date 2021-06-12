import pyvjoy
import math
import time
import socket
import struct
import math
import select
from transformations import *
from configs import *

THR_RATE = 0.7
UDP_IP = "127.0.0.1"
UDP_PORT = 4242


import socket
import re
import numpy as np

DCS_UDP_IP = "127.0.0.1"
DCS_UDP_PORT = 27015
DEFAULT_FOV = 67.4/57.3
VIEW_OFFSET = 0
MAX_BANK = 80/57.3

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
        self.dcs_sock.setblocking(0)

        self.OK = False
        self.updated = False

    def update(self):
        ready = select.select([self.dcs_sock], [], [], 0.002)
        if ready[0]:
            msg, addr = self.dcs_sock.recvfrom(1024) # buffer size is 1024 bytes
            self.data = self.parse_data(msg)
            if not self.OK:
                self.OK = True
                print("DCS Ready")
            self.updated = True

    def parse_data(self, data):
        data = data.decode("utf-8") 
        m = re.findall(r'([a-zA-Z]+)=(-?\d+(\.\d*)?)', data)
        values = {}
        for k, v, _ in m:
            v = float(v)
            values[k] = v
        return values

class game_aircraft_control():
    def __init__(self, win_w, win_h):
        self.vjoyman = VJoyManager()
        self.tracker = GameTracker()
        self.telem = DCSTelem()
        
        self.ail = 0
        self.ele = 0
        self.rud = 0
        self.thr = 0.7

        self.view_yaw = 0
        self.view_pitch = 0

        self.q_view_abs = [1, 0, 0, 0]
        self.q_att_sp = [1, 0, 0, 0]
        self.q_att = [1, 0, 0, 0]

        self.yaw_sp = None
        self.pitch_sp = None
        self.roll_sp = None

        #Note w means world frame
        self.yawrate_w_sp = 0

        self.yawrate_b_sp = 0
        self.pitchrate_b_sp = 0
        self.rollrate_b_sp = 0

        #View camera parameter
        self.cx = win_w /2
        self.cy = win_h /2
        self.fx = self.fy = win_w/2/math.tan(DEFAULT_FOV/2)

        self.is_free_look = True

        print(f"Camera cx {self.cx} cy{self.cy} fx {self.fx} fy {self.fy}")

    def get_ail(self):
        return self.ail

    def get_ele(self):
        return self.ele
    
    def get_rud(self):
        return self.rud
    
    def get_thr(self):
        return self.thr

    def set_mouse_aircraft_control(self, _x, _y):
        x_sp =  _x / screen_scale 
        y_sp =  _y / screen_scale 
        
        if control_style == "battlefield":
            self.ail = x_sp*ailrate
            self.ele = -y_sp*elerate
        elif control_style == "warthunder" and self.OK and self.updated:
            if self.yaw_sp == None:
                self.yaw_sp = self.telem.data['yaw']
                self.pitch_sp = self.telem.data['pitch']
                self.roll_sp = self.telem.data['roll']
            self.yaw_sp += math.atan(_x/self.fx)
            self.pitch_sp += -math.atan(_y/self.fy)
    
    def move_aim_mouse(self):
        if self.pitch_sp is not None:
            # _y = math.tan(-(self.pitch_sp - self.view_pitch))*self.fy
            # _x = math.tan((self.yaw_sp - self.view_yaw))*self.fy
            _y = 0
            _x = 0
            return _x, _y
        else:
            return 0, 0

    def control_body_aim(self, yaw_sp, pitch_sp):
        #need to update to quaternion
        # if not self.is_free_look:
        #     new_view_yaw = wrap_pi(self.yaw_sp)*57.3
        #     self.view_yaw = new_view_yaw*(1-self.view_filter_rate)+self.view_yaw*self.view_filter_rate
        #     new_view_pitch = wrap_pi(self.pitch_sp)*57.3
        #     self.view_pitch = new_view_pitch*(1-self.view_filter_rate)+self.view_pitch*self.view_filter_rate

        # oy = 0
        # oy = -_y/ screen_scale*57.3*view_rate
        # oz =  _x/ screen_scale*57.3*view_rate
        # self.q_view_abs += 0.5*quaternion_multiply(self.q_view_abs, [0, 0, oy, oz])
        # self.q_view_abs = unit_vector(self.q_view_abs)

        dyaw = self.yaw_sp - self.telem.data["yaw"]
        dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi

        #We may also consider use g instead
        self.yawrate_w_sp = dyaw * p_yaw
        self.yawrate_b_sp = self.yawrate_w_sp*math.cos(self.telem.data['pitch'])*math.cos(self.telem.data['roll'])
        self.pitchrate_b_sp = self.yawrate_w_sp*math.cos(self.telem.data['pitch'])*math.sin(self.telem.data['roll'])
        self.roll_sp = float_constrain(self.yawrate_w_sp*p_yawrate_w_to_roll, -MAX_BANK, MAX_BANK)

    def status(self):
        if self.telem.OK:
            _s =  f"yaw: sp {self.yaw_sp*57.3:3.1f} raw {self.telem.data['yaw']*57.3:3.1f} rate_w_sp {self.yawrate_w_sp*57.3:3.1f} rate {self.telem.data['yawrate']*57.3:3.1f}\n"
            _s += f"pitch sp: {self.pitch_sp*57.3:3.1f} pitch {self.telem.data['pitch']*57.3:3.1f} rate_b_sp {self.pitchrate_b_sp*57.3:3.1f} rate {self.telem.data['pitchrate']*57.3:3.1f}\n"
            _s += f"roll: sp {self.roll_sp*57.3:3.1f} raw {self.telem.data['roll']*57.3:3.1f} rate {self.telem.data['rollrate']*57.3:3.1f}"
            return _s
        return "Wait for connection"

    def controller_update(self):
        if self.telem.OK:
            if control_style == "warthunder":
                self.control_body_aim(self.yaw_sp, self.pitch_sp)
                self.ail = (self.roll_sp - self.telem.data["roll"])*p_roll + p_rollrate*(self.rollrate_b_sp-self.telem.data["rollrate"])
                self.ele = (self.pitch_sp - self.telem.data["pitch"])*p_pitch + p_pitchrate*(self.pitchrate_b_sp-self.telem.data["pitchrate"])
                self.rud = p_yawrate*(self.yawrate_b_sp-self.telem.data["pitchrate"])
                # self.ele = self.p_pitchrate*(self.pitchrate_b_sp-self.telem.data["pitchrate"])

                # self.ail = self.roll_sp - self.p_rollrate*self.telem.data["rollrate"]
                # print()
                # print(f"rollsp {self.roll_sp*57.3:3.1f} roll {self.telem.data['roll']*57.3:3.1f} rollrate {self.telem.data['rollrate']*57.3:3.1f}  ail {self.ail}")
                # print(f"pitch_sp {self.pitch_sp} pitch {self.telem.data['pitch']} ele {self.ele}")
        self.telem.updated = False

    def set_mouse_free_look(self, _x, _y):
        self.is_free_look = True
        self.view_yaw += _x/ screen_scale*57.3*view_rate
        self.view_pitch += -_y/ screen_scale*57.3*view_rate
        self.q_view_abs = quaternion_from_euler(0, self.view_pitch, self.view_yaw)
    
    def set_mouse_free_look_off(self):
        self.is_free_look = False
        self.view_yaw = self.telem.data["yaw"]
        self.view_pitch = self.telem.data["pitch"]
        self.q_view_abs = quaternion_from_euler(0, self.view_pitch, self.view_yaw)
        self.q_view_abs = [1, 0, 0, 0]

    def inc_thr(self, dt):
        self.thr += dt*THR_RATE
        if self.thr > 1:
            self.thr = 1

    def dec_thr(self, dt):
        self.thr -= dt*THR_RATE
        if self.thr < 0:
            self.thr = 0

    def pre_update(self):
        self.telem.update()
        self.OK = self.telem.OK
        self.updated = self.telem.updated
        if self.OK and self.updated:
            return self.move_aim_mouse()
        return 0, 0

    def set_camera_view(self):
        euler = euler_from_quaternion(self.q_view_abs)
        # print(self.view_yaw, self.view_pitch, euler)
        # self.tracker.send_pose([self.view_yaw-self.telem.data["yaw"], self.view_pitch - self.telem.data["pitch"]+VIEW_OFFSET, 0], [0, 0, 0])
        self.tracker.send_pose([euler[2]*57.3, euler[1]*57.3, 0], [0, 0, 0])

    def update(self):
        self.controller_update()
        # self.vjoyman.set_joystick_x(self.ail)
        # self.vjoyman.set_joystick_y(self.ele)
        # self.vjoyman.set_joystick_rz(self.rud)
        # self.vjoyman.set_joystick_z(-self.thr)
        self.set_camera_view()

if __name__ == '__main__':
    aircraft_con = game_aircraft_control()

    t = 0
    while True:
        aircraft_con.update()
        t+= 0.01
        time.sleep(0.01)