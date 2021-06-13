import math
import time
import socket
import math
import select
from transformations import *
from configs import *
from utils import *
import socket
import re

if USE_VJOY:
    import pyvjoy

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
        for i in range(3):
            for j in range(3):
                _s += f"{self.R_cam[i, j]:3.4f},"
        for i in range(3):
            _s += f"{self.T_cam[i] :3.4f},"
        
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
        ready = select.select([self.dcs_sock], [], [], 0.002)
        if ready[0]:
            msg, addr = self.dcs_sock.recvfrom(1024) # buffer size is 1024 bytes
            self.data = self.parse_data(msg)
            for k in self.data:
                setattr(self, k, self.data[k])

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

class game_aircraft_control():
    def __init__(self, win_w, win_h):
        if USE_VJOY:
            self.vjoyman = VJoyManager()
        self.tracker = GameTracker()
        self.telem = DCSTelem()
        
        self.ail = 0
        self.ele = 0
        self.rud = 0
        self.thr = 0.7

        self.ail_user = 0
        self.ele_user = 0
        self.rud_user = 0

        self.view_yaw = 0
        self.view_pitch = 0

        self.q_view_abs = None

        self.q_att_sp = np.array([1, 0, 0, 0], dtype=float) # Control Setpoint now, contain roll

        self.q_att_tgt = np.array([1, 0, 0, 0], dtype=float) # Control target, no roll

        self.q_att = np.array([1, 0, 0, 0], dtype=float)

        self.q_cam_pitch = quaternion_from_euler(0, CAM_PITCH_OFFSET, 0)

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

        self.is_free_look = False

        self.last_free_look = False

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
                self.yaw_sp = self.telem.yaw
                self.pitch_sp = self.telem.pitch
                self.roll_sp = self.telem.roll
                self.q_att_tgt = quaternion_from_euler(0, self.pitch_sp, self.yaw_sp)

            ox = 0
            oy = -_y/self.fx*att_sp_rate
            oz =  _x/self.fy*att_sp_rate
            self.q_att_tgt += 0.5*quaternion_multiply(self.q_att_tgt, [0, ox, oy, oz])
            self.q_att_tgt = unit_vector(self.q_att_tgt)
            self.roll_sp, self.pitch_sp, self.yaw_sp = euler_from_quaternion(self.q_att_tgt)
            # self.yaw_sp, self.pitch_sp, self.roll_sp = euler_from_quaternion(self.q_att_tgt, "szyx")

    def move_aim_mouse(self):
        if self.pitch_sp is not None:
            rel_tgt = quaternion_multiply(quaternion_inverse(self.q_view_abs), self.q_att_tgt)
            rel_tgt = quaternion_multiply(rel_tgt, self.q_cam_pitch)
            mat = quaternion_matrix(rel_tgt)[0:3,0:3]
            v = np.dot(mat, [1, 0, 0])
            v = v / v[0]
            return v[1]*self.fx, v[2]*self.fy
        else:
            return 0, 0
    
    def move_aim_tgt(self):
        if self.pitch_sp is not None:
            rel_tgt = quaternion_multiply(quaternion_inverse(self.q_view_abs), self.q_att)
            rel_tgt = quaternion_multiply(rel_tgt, self.q_cam_pitch)
            mat = quaternion_matrix(rel_tgt)[0:3,0:3]
            v = np.dot(mat, [1, 0, 0])
            v = v / v[0]
            return v[1]*self.fx, v[2]*self.fy
        else:
            return 0, 0

    def control_body_aim(self, q_tgt):
        #need to update to quaternion
        if not self.is_free_look:
            self.q_view_abs = quaternion_slerp(self.q_view_abs, self.q_att_tgt, view_filter_rate)


        dyaw = self.yaw_sp - self.telem.data["yaw"]
        dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi

        #We may also consider use g instead
        self.yawrate_w_sp = dyaw * p_yaw
        self.yawrate_b_sp = self.yawrate_w_sp*math.cos(self.telem.pitch)*math.cos(self.telem.roll)
        
        self.pitchrate_b_sp = self.yawrate_w_sp*math.cos(self.telem.pitch)*math.sin(self.telem.roll)
        self.roll_sp = float_constrain(self.yawrate_w_sp*p_yawrate_w_to_roll, -MAX_BANK, MAX_BANK)

        self.q_att_sp = quaternion_from_euler(self.roll_sp, self.pitch_sp, self.yaw_sp)

    def status(self):
        if self.telem.OK:
            _s = f"{self.telem.name} TAS: {self.telem.tas*3.6:3.1f}km/h Aoa {self.telem.aoa:3.1f}\n"
            _s += f"yaw: sp {self.yaw_sp*57.3:3.1f} raw {self.telem.yaw*57.3:3.1f} rate_w_sp {self.yawrate_w_sp*57.3:3.1f} rate {self.telem.data['yawrate']*57.3:3.1f}\n"
            _s += f"pitch sp: {self.pitch_sp*57.3:3.1f} pitch {self.telem.pitch*57.3:3.1f} rate_b_sp {self.pitchrate_b_sp*57.3:3.1f} rate {self.telem.data['pitchrate']*57.3:3.1f}\n"
            _s += f"roll: sp {self.roll_sp*57.3:3.1f} raw {self.telem.roll*57.3:3.1f} rate {self.telem.data['rollrate']*57.3:3.1f}\n"
            return _s
        return "Wait for connection"

    def controller_update(self):
        if self.telem.OK:
            if control_style == "warthunder":
                self.control_body_aim(self.q_att_tgt)
                self.ail = (self.roll_sp - self.telem.data["roll"])*p_roll + p_rollrate*(self.rollrate_b_sp-self.telem.data["rollrate"])
                self.ele = (self.pitch_sp - self.telem.data["pitch"])*p_pitch + p_pitchrate*(self.pitchrate_b_sp-self.telem.data["pitchrate"])
                self.rud = p_yawrate*(self.yawrate_b_sp-self.telem.data["pitchrate"])
                # self.ele = self.p_pitchrate*(self.pitchrate_b_sp-self.telem.data["pitchrate"])

                # self.ail = self.roll_sp - self.p_rollrate*self.telem.data["rollrate"]
                # print()
                # print(f"rollsp {self.roll_sp*57.3:3.1f} roll {self.telem.roll*57.3:3.1f} rollrate {self.telem.data['rollrate']*57.3:3.1f}  ail {self.ail}")
                # print(f"pitch_sp {self.pitch_sp} pitch {self.telem.pitch} ele {self.ele}")
        self.telem.updated = False

    def set_mouse_free_look(self, _x, _y):
        self.is_free_look = True
        self.view_yaw += _x/ screen_scale*57.3*view_rate
        self.view_pitch += -_y/ screen_scale*57.3*view_rate
        self.q_view_abs = quaternion_from_euler(0, self.view_pitch, self.view_yaw)
        self.last_free_look = True
    
    def set_mouse_free_look_off(self):
        self.is_free_look = False
        if self.q_view_abs is None or self.last_free_look:
            self.view_yaw = self.telem.data["yaw"]
            self.view_pitch = self.telem.data["pitch"]
            self.q_view_abs = quaternion_from_euler(0, self.view_pitch, self.view_yaw)
            self.last_free_look = False

    def set_user_ele(self, ele):
        self.user_ele = ele 

    def set_user_ail(self, ail):
        self.user_ail = ail 

    def set_user_rud(self, rud):
        self.user_rud = rud

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

        self.user_ail = None
        self.user_ele = None
        self.user_rud = None

        if self.OK and self.updated:
            self.q_att = quaternion_from_euler(self.telem.data["roll"], self.telem.data["pitch"], self.telem.data["yaw"])
            _1, _2 = self.move_aim_mouse()
            _3, _4 = self.move_aim_tgt()
            return _1, _2, _3, _4
        return 0, 0, 0, 0

    def set_camera_view(self):
        q_rel = quaternion_multiply(quaternion_inverse(self.q_att), self.q_view_abs)
        euler = euler_from_quaternion(q_rel)
        self.tracker.send_pose([euler[2]*57.3, euler[1]*57.3, 0], [0, 0, 0])

    def cameraPose(self):
        T_cam =  np.array([self.telem.x, self.telem.y, self.telem.z])
        mat = quaternion_matrix(quaternion_inverse(self.q_view_abs))[0:3,0:3]
        T_cam += np.dot(mat, [CAMERA_X, 0, CAMERA_Z])

        return self.q_view_abs, T_cam

    def update(self):
        self.controller_update()
        ail = float_constrain(self.user_ail if self.user_ail else self.ail, -1, 1)
        ele = float_constrain(self.user_ele if self.user_ele else self.ele, -1, 1)
        rud = float_constrain(self.user_rud if self.user_rud else self.rud, -1, 1)
        
        if USE_VJOY:
            self.vjoyman.set_joystick_x(ail)
            self.vjoyman.set_joystick_y(ele)
            self.vjoyman.set_joystick_rz(rud)
            self.vjoyman.set_joystick_z(-self.thr)
            self.set_camera_view()
        else:
            q_cam, T_cam = self.cameraPose()
            self.telem.set_camera_pose(q_cam, T_cam)
            self.telem.set_control(ail, ele, rud, self.thr)
            self.telem.send_dcs_command()

if __name__ == '__main__':
    aircraft_con = game_aircraft_control()

    t = 0
    while True:
        aircraft_con.update()
        t+= 0.01
        time.sleep(0.01)