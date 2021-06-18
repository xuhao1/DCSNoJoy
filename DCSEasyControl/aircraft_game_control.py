import time
import math
from transformations import *

from .params_server import ParameterServer
from Configs.configs import *
from .utils import *
from .DCSTelem import *
from .dcs_cam_control import *
from .flight_controller import *
from yaml import load, dump


class game_aircraft_control():
    def __init__(self, win_w, win_h, path):
        self.telem = DCSTelem()
        self.OK = False
        self.param = ParameterServer(path)
        self.param.load_aircraft("default")
        self.cam = dcs_cam_control(win_w, win_h, self, self.param)
        self.fc = FlightController(self, self.param)
        self.thr = 0.9

        #View camera parameter
        self.reset()

    def reset(self):
        print("Reset aircraft con")
        self.ail = 0
        self.ele = 0
        self.rud = 0

        self.ail_user = 0
        self.ele_user = 0
        self.rud_user = 0

        self.fc.reset()
        self.cam.reset()
        self.log = ""

        if self.telem.OK:
            self.param.load_aircraft(self.telem.name)

    def get_ail(self):
        return self.ail

    def get_ele(self):
        return self.ele
    
    def get_rud(self):
        return self.rud
    
    def get_thr(self):
        return self.thr

    def set_mouse_aircraft_control(self, _x, _y):
        x_sp =  _x / self.cam.fx 
        y_sp =  _y / self.cam.fx 
        
        if control_style == "battlefield":
            self.ail += x_sp*ailrate
            self.ele += -y_sp*elerate
        elif control_style == "warthunder" and self.OK and self.updated:
            dir_tgt = self.fc.get_dir_tgt() + quaternion_rotate(self.cam.q_view_abs, np.array([0, x_sp, y_sp], dtype=float))
            dir_tgt = unit_vector(dir_tgt)
            self.fc.set_dir_tgt(dir_tgt)

    def dir_to_screenpos(self, dir):
        if ACTIVE_CTRL_F3:
            v = quaternion_rotate(quaternion_inverse(self.telem.q_telem_cam), dir)
        else:
            v = quaternion_rotate(quaternion_inverse(self.cam.q_view_abs), dir)
        v = v / v[0]
        return v[1]*self.cam.fx, v[2]*self.cam.fy

    def move_aim_mouse(self):
        if self.fc.pitch_sp is not None:
            return self.dir_to_screenpos(self.get_dir_tgt())
        return -10000, -10000
    
    def move_aim_tgt(self):
        if self.fc.pitch_sp is not None:
            q = quaternion_multiply(self.fc.q_att, quaternion_from_euler(0, self.param.gun_pitch/57.3, 0))
            return self.dir_to_screenpos(q_to_dir(q))
        return -10000, -10000

    def get_dir_tgt(self):
        return self.fc.get_dir_tgt()

    def status(self):
        yaw_sp, pitch_sp, roll_sp = self.fc.yaw_sp, self.fc.pitch_sp, self.fc.roll_sp
        dw_sp = self.fc.dw_sp
        dv = self.fc.dv
        N_sp_w = self.fc.N_sp_w
        Nz_sp = self.fc.Nz_sp
        dir_tgt_b = self.fc.dir_tgt_b

        if self.telem.OK:
            _s =  f"t\t{self.telem.time:3.1f}\t{self.telem.name}\n"
            _s += f"TAS:\t{self.telem.tas*3.6:3.1f}km/h\tAoA:{self.telem.aoa:3.1f}deg\n"
            _s += f"yaw: \tsp\t{yaw_sp*57.3:3.1f}\traw {self.telem.yaw*57.3:3.1f}\terr {dw_sp[0]*57.3:3.1f}\trate {self.telem.yawrate*57.3:3.1f}\trud {self.rud*100:3.1f}%\n"
            _s += f"pitch \tsp\t{pitch_sp*57.3:3.1f}\traw {self.telem.pitch*57.3:3.1f}\terr {dw_sp[1]*57.3:3.1f}\trate {self.telem.pitchrate*57.3:3.1f}\tele {self.ele*100:3.1f}%\n"
            _s += f"roll: \tsp\t{roll_sp*57.3:3.1f}\traw {self.telem.roll*57.3:3.1f}\terr {dw_sp[2]*57.3:3.1f}\trate {self.telem.rollrate*57.3:3.1f}\tail {self.ail*100:3.1f}%\n"
            _s += f"thr\t{self.thr*100:5.1f}%\n"
            _s += f"dv \t{dv[0]:3.1f}\t{dv[1]:3.1f}\t{dv[2]:3.1f}\tdir_tgt_b {dir_tgt_b[0]:3.1f}\t{dir_tgt_b[1]:3.1f}\t{dir_tgt_b[2]:3.1f}\n"
            _s += f"load {self.telem.Nz:3.1f}g -Nz_sp\t{-self.fc.Nz_sp/G:3.1f}g\t NSp\t{N_sp_w[0]/G:3.1f}\t{N_sp_w[1]/G:3.1f}\t{N_sp_w[2]/G:3.1f}g\tdload {self.telem.Nz - (-Nz_sp/G):3.1f}g\n\n Log:"
            _s += self.log
            print(_s)
            return _s
        return "Wait for connection"


    def controller_update(self, dt):
        if self.telem.OK:
            self.fc.control(dt)
        self.telem.updated = False

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
        if self.telem.OK and not self.OK:
            self.OK = self.telem.OK
            self.reset()

        self.OK = self.telem.OK
        self.updated = self.telem.updated

        self.user_ail = None
        self.user_ele = None
        self.user_rud = None

        if self.OK and self.updated:
            q_att = quaternion_from_euler(self.telem.data["roll"], self.telem.data["pitch"], self.telem.data["yaw"])
            self.fc.set_att(q_att)
            _1, _2 = self.move_aim_mouse()
            _3, _4 = self.move_aim_tgt()
            return _1, _2, _3, _4
        return 0, 0, 0, 0
    
    def update(self):
        if not self.telem.OK or not self.updated:
            return
        self.controller_update(0.01)
        self.ail = float_constrain(self.user_ail if self.user_ail else self.fc.ail, -1, 1)
        self.ele = float_constrain(self.user_ele if self.user_ele else self.fc.ele, -1, 1)
        self.rud = float_constrain(self.user_rud if self.user_rud else self.fc.rud, -1, 1)
        
        self.cam.set_camera_view()
        self.telem.set_control(self.ail, self.ele, self.rud,(self.thr * 2 -1))
        self.telem.send_dcs_command()

if __name__ == '__main__':
    aircraft_con = game_aircraft_control()

    t = 0
    while True:
        aircraft_con.update()
        t+= 0.01
        time.sleep(0.01)