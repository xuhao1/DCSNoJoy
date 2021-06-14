import math
import time
import socket
import math
from transformations import *
from configs import *
from utils import *
from DCSTelem import *
import socket

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


class game_aircraft_control():
    def __init__(self, win_w, win_h):
        if USE_VJOY:
            self.vjoyman = VJoyManager()
        self.tracker = GameTracker()
        self.telem = DCSTelem()
        
        self.ail = 0
        self.ele = 0
        self.rud = 0
        self.thr = 0.9

        self.ail_user = 0
        self.ele_user = 0
        self.rud_user = 0

        self.view_yaw = 0
        self.view_pitch = 0

        self.q_view_abs = None

        self.q_att_sp = np.array([1, 0, 0, 0], dtype=float) # Control Setpoint now, contain roll

        self.q_att_tgt = np.array([1, 0, 0, 0], dtype=float) # Control target, no roll
        self.dir_att = np.array([1, 0, 0])

        self.q_att = np.array([1, 0, 0, 0], dtype=float)

        self.q_cam_pitch_offset = quaternion_from_euler(0, CAM_PITCH_OFFSET, 0)

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
            oy =  _y/self.fx*att_sp_rate
            oz =  _x/self.fy*att_sp_rate
            #First we need to rotate them by camera frame
            dw = np.array([0, ox, oy, oz])
            dw = quaternion_multiply(self.q_view_abs, dw)
            self.q_att_tgt += 0.5*quaternion_multiply(dw, self.q_att_tgt)
            self.q_att_tgt = setZeroRoll(self.q_att_tgt)
            self.roll_sp, self.pitch_sp, self.yaw_sp = euler_from_quaternion(self.q_att_tgt)
            # self.yaw_sp, self.pitch_sp, self.roll_sp = euler_from_quaternion(self.q_att_tgt, "szyx")

    def move_aim_mouse(self):
        if self.pitch_sp is not None:
            rel_tgt = quaternion_multiply(quaternion_inverse(self.q_view_abs), self.q_att_tgt)
            if USE_OPENTRACK:
                rel_tgt = quaternion_multiply(rel_tgt, self.q_cam_pitch_offset)
            v = quaternion_rotate(rel_tgt, np.array([1, 0, 0]))
            v = v / v[0]
            return v[1]*self.fx, v[2]*self.fy
        else:
            return 0, 0
    
    def move_aim_tgt(self):
        if self.pitch_sp is not None:
            rel_tgt = quaternion_multiply(quaternion_inverse(self.q_view_abs), self.q_att)
            if USE_OPENTRACK:
                rel_tgt = quaternion_multiply(rel_tgt, self.q_cam_pitch_offset)
            v = quaternion_rotate(rel_tgt, np.array([1, 0, 0]))
            v = v / v[0]
            return v[1]*self.fx, v[2]*self.fy
        else:
            return 0, 0


    def control_body_aim(self, q_tgt):
        #need to update to quaternion
        dw = att_err_to_tangent_space(self.q_att_tgt, self.q_att)
        dyaw = dw[2]

        #We may also consider use g instead
        self.yawrate_w_sp = dyaw * p_yaw
        self.roll_sp = float_constrain(self.yawrate_w_sp*p_yawrate_w_to_roll, -MAX_BANK, MAX_BANK)
        self.q_att_sp = quaternion_from_euler(self.roll_sp, self.pitch_sp, self.yaw_sp)

        dw = att_err_to_tangent_space(self.q_att_sp, self.q_att)

        return dw


    def status(self):
        # yaw_sp, pitch_sp, roll_sp = euler_from_quaternion(self.q_att_tgt, "rzyx")
        yaw_sp, pitch_sp, roll_sp = self.yaw_sp, self.pitch_sp, self.roll_sp

        if self.telem.OK:
            _s = f"{self.telem.name} t {self.telem.time:3.1f}\n"
            _s += f"TAS: {self.telem.tas*3.6:3.1f}km/h AoA {self.telem.aoa:3.1f}\n"
            _s += f"yaw: sp {yaw_sp*57.3:3.1f} raw {self.telem.yaw*57.3:3.1f} rate_w_sp {self.yawrate_w_sp*57.3:3.1f} rate {self.telem.data['yawrate']*57.3:3.1f}\n"
            _s += f"pitch sp: {pitch_sp*57.3:3.1f} pitch {self.telem.pitch*57.3:3.1f} rate_b_sp {self.pitchrate_b_sp*57.3:3.1f} rate {self.telem.data['pitchrate']*57.3:3.1f}\n"
            _s += f"roll: sp {roll_sp*57.3:3.1f} raw {self.telem.roll*57.3:3.1f} rate {self.telem.data['rollrate']*57.3:3.1f}\n"
            # _s += f"x {self.telem.x:5.1f} y {self.telem.y:3.1f} z {self.telem.z:3.1f}\n"
            _s += f"thr {self.thr*100:5.1f}%"
            print(_s)
            return _s
        return "Wait for connection"

    def controller_update(self):
        if self.telem.OK:
            if control_style == "warthunder":
                dw = self.control_body_aim(self.q_att_tgt)
                self.ail = (dw[0])*p_roll + p_rollrate*(self.rollrate_b_sp-self.telem.data["rollrate"])
                self.ele = (dw[1])*p_pitch + p_pitchrate*(self.pitchrate_b_sp-self.telem.data["pitchrate"])
                self.rud = p_yawrate*(self.yawrate_b_sp-self.telem.data["pitchrate"])

        self.telem.updated = False

    def q_default_view(self):
        q_view_sp = quaternion_multiply(self.q_cam_pitch_offset, self.q_att_sp)
        q_view_sp = setZeroRoll(q_view_sp)
        return q_view_sp

    def set_mouse_free_look(self, _x, _y):
        self.is_free_look = True
        ox = 0
        oy = -_y/self.fx*view_rate
        oz =  _x/self.fy*view_rate
        self.q_view_abs += 0.5*quaternion_multiply([0, ox, oy, oz], self.q_view_abs)
        self.q_view_abs = setZeroRoll(self.q_view_abs)
        _, self.view_pitch, self.view_yaw = euler_from_quaternion(self.q_view_abs)
        print(f"Free look yaw {self.view_yaw*57.3:3.1f} pitch {self.view_pitch*57.3:3.1f} ")
        self.last_free_look = True
    
    def set_mouse_free_look_off(self):
        self.is_free_look = False
        if self.q_view_abs is None or self.last_free_look:
            self.q_view_abs = self.q_default_view()
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
        if not self.is_free_look:
            q_view_sp = self.q_default_view()
            self.q_view_abs = quaternion_slerp(self.q_view_abs, q_view_sp, view_filter_rate)
        _, self.view_pitch, self.view_yaw = euler_from_quaternion(self.q_view_abs)

        if USE_OPENTRACK:
            q_rel = quaternion_multiply(quaternion_from_euler(0, -self.telem.pitch, -self.telem.yaw), self.q_view_abs)
            euler = euler_from_quaternion(q_rel)

            mat = quaternion_matrix(quaternion_inverse(self.q_view_abs))[0:3,0:3]
            T_cam = np.dot(mat, [CAMERA_X, 0, CAMERA_Z])*100
            
            self.tracker.send_pose([euler[2]*57.3, euler[1]*57.3, 0], [0, 0, 0])
        else:
            q_cam, T_cam = self.cameraPose()
            self.telem.set_camera_pose(self.view_yaw, self.view_pitch, T_cam)

    def cameraPose(self):
        # T is relative to our aircraft
        # mat = quaternion_matrix(quaternion_inverse(self.q_view_abs))[0:3,0:3]
        mat = quaternion_matrix(self.q_view_abs)[0:3,0:3]
        T_cam = mat @ [-CAMERA_X, 0, -CAMERA_Z]
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
            self.vjoyman.set_joystick_z(-(self.thr * 2 -1))
            self.set_camera_view()
        else:
            self.set_camera_view()

            self.telem.set_control(ail, ele, rud,(self.thr * 2 -1))
            self.telem.send_dcs_command()

if __name__ == '__main__':
    aircraft_con = game_aircraft_control()

    t = 0
    while True:
        aircraft_con.update()
        t+= 0.01
        time.sleep(0.01)