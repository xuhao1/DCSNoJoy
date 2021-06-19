import socket
from transformations import *
from Configs.configs import *
from .utils import *
from .DCSTelem import *
import socket



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

if USE_VJOY:
    import pyvjoy
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
