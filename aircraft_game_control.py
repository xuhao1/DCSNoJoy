import pyvjoy
import math
import time
import socket
import struct

THR_RATE = 0.7
UDP_IP = "127.0.0.1"
UDP_PORT = 4242

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

class game_aircraft_control():
    def __init__(self):
        self.vjoyman = VJoyManager()
        self.tracker = GameTracker()

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

    def get_ail(self):
        return self.ail

    def get_ele(self):
        return self.ele
    
    def get_rud(self):
        return self.rud
    
    def get_thr(self):
        return self.thr

    def set_mouse_aircraft_control(self, _x, _y):
        self.ail = _x / self.screen_scale *self.ailrate
        self.ele = -_y / self.screen_scale*self.elerate
    
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
        self.vjoyman.set_joystick_x(self.ail)
        self.vjoyman.set_joystick_y(self.ele)
        self.vjoyman.set_joystick_rz(self.rud)
        self.vjoyman.set_joystick_z(-self.thr)
        self.tracker.send_pose([self.view_yaw, self.view_pitch, 0], [0, 0, 0])

if __name__ == '__main__':
    vjoy = VJoyManager()

    t = 0
    while True:
        vjoy.set_joystick_x(math.sin(t))
        t+= 0.01
        time.sleep(0.01)