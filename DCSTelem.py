import socket
import re
import select
from configs import *
import numpy as np
from transformations import *

class DCSTelem():
    def __init__(self):
        print("Trying to connect to DCS Telem")
        self.dcs_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        self.dcs_sock.bind((DCS_UDP_IP, DCS_UDP_PORT))
        self.dcs_sock.setblocking(0)

        self.dcs_sock_send = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

        self.OK = False
        self.updated = False

        self.R_cam = np.eye(3, 3)
        self.q_cam = np.array([1, 0, 0, 0])
        self.T_cam = np.array([0, 0, 0])

        self.ail = 0
        self.ele = 0
        self.rud = 0
        self.thr = 0

    def send_dcs(self, data):
        self.dcs_sock_send.sendto(data.encode(), (DCS_UDP_IP, DCS_UDP_SEND_PORT))

    def send_dcs_command(self):
        #0-9 camera R, R01 R02.. R22
        #9-12 camera T
        #12-16 ail ele rud thr
        _s = f"{self.time:3.4f},"
        _s += f"{self.ail:3.4f},"
        _s += f"{self.ele:3.4f},"
        _s += f"{self.rud:3.4f},"
        _s += f"{self.thr:3.4f},"

        _s += f"{self.T_cam[0]:3.4f},"
        _s += f"{self.T_cam[1]:3.4f},"
        _s += f"{self.T_cam[2]:3.4f},"

        for i in range(3):
            for j in range(3):
                _s += f"{self.R_cam[i, j]:3.4f},"
        _s += "\n"
        self.send_dcs(_s)

    def set_camera_pose(self, view_yaw, view_pitch, T):
        # self.R_cam = quaternion_matrix(q)
        self.R_cam = euler_matrix(0, view_yaw, -view_pitch)
        Rcam = self.R_cam[0:3, 0:3]
        self.T_cam[0] = T[0]
        self.T_cam[1] = -T[2]
        self.T_cam[2] = T[1]

    def set_control(self, ail, ele, rud, thr):
        self.ail = ail
        self.ele = ele
        self.rud = rud
        self.thr = thr

    def update(self):
        ready = select.select([self.dcs_sock], [], [], 0.001)
        while ready[0]:
            msg, addr = self.dcs_sock.recvfrom(1024) # buffer size is 1024 bytes
            ready = select.select([self.dcs_sock], [], [], 0.001) #Recv last

            self.data = self.parse_data(msg)
            for k in self.data:
                setattr(self, k, self.data[k])

            # convert origin Rcam to euler will give roll yaw pitch, so we need to switch z y axis
            
            if not self.OK:
                self.OK = True
                print("DCS Ready")
            self.updated = True

    def parse_data(self, data):
        data = data.decode("utf-8") 
        # m = re.findall(r'([a-zA-Z]+)=(-?\d+(\.\d*)?)', data)
        m = re.findall(r'([a-zA-Z]+)=(-?\d+(\.\d*)?|\S+)', data)
        values = {}
        for k, v, _ in m:
            if k != "name":
                v = float(v)
            values[k] = v

        return values