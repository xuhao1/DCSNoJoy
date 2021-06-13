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
        self.T_cam = np.eye(3)

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
        _s = ""
        _s += f"{self.time:3.4f},"
        _s += f"{self.ail:3.4f},"
        _s += f"{self.ele:3.4f},"
        _s += f"{self.rud:3.4f},"
        _s += f"{self.thr:3.4f}\n"

        self.send_dcs(_s)

    def set_camera_pose(self, q, T):
        self.R_cam = quaternion_matrix(q)
        self.T_cam = T
        print("cam Pose", self.T_cam)

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

            self.R_cam = np.array([
                [self.Rcamxx, self.Rcamxy, self.Rcamxz, 0],
                [self.Rcamyx, self.Rcamyy, self.Rcamyz, 0],
                [self.Rcamzx, self.Rcamzy, self.Rcamzz, 0],
                [0, 0, 0, 1]
            ])
            # convert origin Rcam to euler will give roll yaw pitch, so we need to switch z y axis
            R_cam = self.R_cam
            q_cam = quaternion_from_matrix(R_cam)
            r, p, y = euler_from_quaternion(q_cam, "sxyz")
            # print("Rcam0 ypr", y*57.3, p*57.3, r*57.3)
            self.q_cam = q_cam
            
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