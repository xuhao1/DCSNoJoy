import math
import time
import math
from transformations import *
from configs import *
from utils import *
from DCSTelem import *
from dcs_cam_control import *

class PIDController:
    def __init__(self, p, i, d, lim_int = 1.0):
        self.p = p
        self.i = i
        self.d = d
        self.err_int = 0
        self.err_last = None
        if i > 0:
            self.lim_int = lim_int/i
        else:
            self.lim_int = 0

    def control(self, err, dt):
        if self.err_last is None:
            self.err_last = err
        self.err_int = float_constrain(err*dt, -self.lim_int, self.lim_int)
        derr = (err-self.err_last)/dt
        self.err_last = err
        return self.p * err + self.i * self.err_int + self.d * derr
    
    def reset(self):
        self.err_int = 0
        self.err_last = None

class game_aircraft_control():
    def __init__(self, win_w, win_h):
        self.telem = DCSTelem()
        self.cam = dcs_cam_control(win_w, win_h, self)
        #View camera parameter
        self.OK = False

        self.g_ele_con = PIDController(p_nz_ele, i_nz_ele, d_nz_ele, lim_ele_int)
        self.pit_ele_con = PIDController(p_pitch, i_pitch, 0, lim_ele_int)

        self.reset()

    def reset(self):
        print("Reset aircraft con")
        self.ail = 0
        self.ele = 0
        self.rud = 0
        self.thr = 0.9

        self.ail_user = 0
        self.ele_user = 0
        self.rud_user = 0

        self.q_att_sp = np.array([1, 0, 0, 0], dtype=float) # Control Setpoint now, contain roll
        self.q_att_tgt = np.array([1, 0, 0, 0], dtype=float) # Control target, no roll
        self.dir_tgt = np.array([1, 0, 0], dtype=float)
        self.dir_sp = np.array([1, 0, 0], dtype=float)
        self.dv = np.array([0, 0, 0], dtype=float)

        self.q_att = np.array([1, 0, 0, 0], dtype=float)


        self.yaw_sp = None
        self.pitch_sp = None
        self.roll_sp = None

        self.N_sp_b = np.array([0, 0, 0], dtype=float)
        self.N_sp_w = np.array([0, 0, 0], dtype=float)
        self.Nz_sp = 0

        #Note w means world frame
        self.yawrate_w_sp = 0

        self.yawrate_b_sp = 0
        self.pitchrate_b_sp = 0
        self.rollrate_b_sp = 0

        self.g_ele_con.reset()

        self.dw_tgt = np.array([0, 0, 0], dtype=float) # attitude now to tgt
        self.dw_sp = np.array([0, 0, 0], dtype=float) # attitude now to sp


        if self.OK and self.yaw_sp == None:
            print("Reset aircraft attitude setpoint")
            self.yaw_sp = self.telem.yaw
            self.pitch_sp = self.telem.pitch
            self.roll_sp = self.telem.roll
            self.q_att_tgt = quaternion_from_euler(0, self.pitch_sp, self.yaw_sp)
            self.dir_tgt = q_to_dir(self.q_att_tgt)
            self.dir_sp = q_to_dir(self.q_att_tgt)
        self.cam.reset()
        self.log = ""

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
            self.dir_tgt += quaternion_rotate(self.cam.q_view_abs, np.array([0, x_sp, y_sp], dtype=float))
            self.dir_tgt = unit_vector(self.dir_tgt)
            self.q_att_tgt = dir_to_q(self.dir_tgt)
            # self.yaw_sp, self.pitch_sp, self.roll_sp = euler_from_quaternion(self.q_att_tgt, "szyx")

    def dir_to_screenpos(self, dir):
        if ACTIVE_CTRL_F3:
            v = quaternion_rotate(quaternion_inverse(self.telem.q_telem_cam), dir)
        else:
            v = quaternion_rotate(quaternion_inverse(self.cam.q_view_abs), dir)
        v = v / v[0]
        return v[1]*self.cam.fx, v[2]*self.cam.fy

    def move_aim_mouse(self):
        if self.pitch_sp is not None:
            return self.dir_to_screenpos(self.dir_tgt)
        return -10000, -10000
    
    def move_aim_tgt(self):
        if self.pitch_sp is not None:
            return self.dir_to_screenpos(self.dir_now)
        return -10000, -10000

    def status(self):
        # yaw_sp, pitch_sp, roll_sp = euler_from_quaternion(self.q_att_tgt, "rzyx")
        yaw_sp, pitch_sp, roll_sp = self.yaw_sp, self.pitch_sp, self.roll_sp

        if self.telem.OK:
            _s =  f"t\t{self.telem.time:3.1f}\t{self.telem.name}\n"
            _s += f"TAS:\t{self.telem.tas*3.6:3.1f}km/h\tAoA:{self.telem.aoa:3.1f}\n"
            _s += f"yaw: \tsp\t{yaw_sp*57.3:3.1f}\traw {self.telem.yaw*57.3:3.1f}\terr {self.dw_sp[0]*57.3:3.1f}\trate {self.telem.yawrate*57.3:3.1f}\trud {self.rud*100:3.1f}%\n"
            _s += f"pitch \tsp\t{pitch_sp*57.3:3.1f}\traw {self.telem.pitch*57.3:3.1f}\terr {self.dw_sp[1]*57.3:3.1f}\trate {self.telem.pitchrate*57.3:3.1f}\tele {self.ele*100:3.1f}%\n"
            _s += f"roll: \tsp\t{roll_sp*57.3:3.1f}\traw {self.telem.roll*57.3:3.1f}\terr {self.dw_sp[2]*57.3:3.1f}\trate {self.telem.rollrate*57.3:3.1f}\tail {self.ail*100:3.1f}%\n"
            _s += f"thr\t{self.thr*100:5.1f}%\n"
            _s += f"dv \t{self.dv[0]:3.1f}\t{self.dv[1]:3.1f}\t{self.dv[2]:3.1f}\n"
            _s += f"load {self.telem.Nz}g -Nz_sp\t{-self.Nz_sp/G:3.1f}g\t NSp\t{self.N_sp_w[0]/G:3.1f}\t{self.N_sp_w[1]/G:3.1f}\t{self.N_sp_w[2]/G:3.1f}g\tdload {self.telem.Nz - (-self.Nz_sp/G):3.1f}g\n\n Log:"
            _s += self.log
            print(_s)
            return _s
        return "Wait for connection"

    def w_to_b(self, v):
        return quaternion_rotate(quaternion_inverse(self.q_att),  v)
    
    def b_to_w(self, v):
        return quaternion_rotate(self.q_att,  v)

    def control_body_aim(self, q_tgt):
        dv = self.w_to_b(self.dir_tgt) - np.array([1, 0, 0])
        self.dv = dv = dv*self.telem.tas
        N_bz = p_dir_nz*np.dot(dv, [0, 0, 1]) # Require load project to z axis
        N_by = p_dir_nz*np.dot(dv, [0, 1, 0])  # Require load parallel to y axis
        self.N_sp_b = np.array([0, N_by, N_bz]) + self.w_to_b([0, 0, -G])
        self.Nz_sp = self.N_sp_b[2]
        self.N_sp_w = self.b_to_w(-self.N_sp_b)

        N_sp_w_v = unit_vector(self.N_sp_w)
        #We should set up the new q_att here by N_sp_w and new vel which vertical to N_sp_w but at same plane with dir now
        self.dir_sp = unit_vector(self.dir_now + self.b_to_w(np.array([0, N_by, N_bz]))/self.telem.tas) #What happen when N_sp_w and dir now is parallel?

        self.q_att_sp = dir_to_q(self.dir_tgt, N_sp_w_v)

        self.roll_sp, self.pitch_sp, self.yaw_sp = euler_from_quaternion(self.q_att_sp)
        # print("dir_sp", self.dir_sp, "N_sp_w_v", N_sp_w_v, "q_att_sp", self.q_att_sp, "q_att", self.q_att, "att rpy", r*57.3, p*57.3, y*57.3, "euler sp", self.roll_sp*57.3, self.pitch_sp*57.3, self.yaw_sp*57.3)
        dw = att_err_to_tangent_space(self.q_att_sp, self.q_att)
        
        self.dw_sp = dw
        return dw

    def controller_update(self, dt):
        if self.telem.OK:
            if control_style == "warthunder":
                if self.telem.tas > min_spd:
                    tas_coeff  = cruise_spd*cruise_spd/(self.telem.tas*self.telem.tas)
                else:
                    tas_coeff  = cruise_spd*cruise_spd/(min_spd*min_spd)

                dw = self.control_body_aim(self.q_att_tgt)
                self.ail = dw[0]*p_roll + p_rollrate*(self.rollrate_b_sp-self.telem.rollrate)
                ele_by_pit = self.pit_ele_con.control(dw[1], dt) 
                ele_by_rate = p_pitchrate*(self.pitchrate_b_sp-self.telem.pitchrate)
                ele_by_g = tas_coeff* self.g_ele_con.control(((-self.Nz_sp/G) - self.telem.Nz), dt)
                self.ele = ele_by_pit+ele_by_rate+ele_by_g
                print(f"""ele_by_pit {ele_by_pit*100:3.1f} ele_by_rate {ele_by_rate*100:3.1f} ele_by_g {ele_by_g*100:3.1f} ele {self.ele*100}
                    (-self.Nz_sp/G) {(-self.Nz_sp/G):3.1f} self.telem.Nz {self.telem.Nz}
                """)
                self.rud = dw[2]*p_yaw + p_yawrate*(self.yawrate_b_sp-self.telem.yawrate)

                # self.log = f"dw {dw[0]*57.3:3.1f} {dw[1]*57.3:3.1f} {dw[2]*57.3:3.1f} ele angle err {dw[1]:3.4f} out {dw[1]*p_pitch*100:3.4f} dampping err {(self.pitchrate_b_sp-self.telem.pitchrate):3.4f}  \
                # output {p_pitchrate*(self.pitchrate_b_sp-self.telem.pitchrate)*100:3.4f} load err {((-self.Nz_sp/G) - self.telem.Nz):3.4f} output {p_nz_ele *tas_coeff* ((-self.Nz_sp/G) - self.telem.Nz):3.4f} "

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
            self.q_att = quaternion_from_euler(self.telem.data["roll"], self.telem.data["pitch"], self.telem.data["yaw"])
            self.dir_now = q_to_dir(self.q_att)
            _1, _2 = self.move_aim_mouse()
            _3, _4 = self.move_aim_tgt()
            return _1, _2, _3, _4
        return 0, 0, 0, 0
    
    def update(self):
        if not self.telem.OK or not self.updated:
            return
        self.controller_update(0.01)
        self.ail = float_constrain(self.user_ail if self.user_ail else self.ail, -1, 1)
        self.ele = float_constrain(self.user_ele if self.user_ele else self.ele, -1, 1)
        self.rud = float_constrain(self.user_rud if self.user_rud else self.rud, -1, 1)
        
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