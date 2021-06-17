import time
import math
from transformations import *
from configs import *
from utils import *
from DCSTelem import *
from dcs_cam_control import *


SMALL_FLOAT = 1e-3
MIN_X_SPHERE = 0.3

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
        self.err_int = float_constrain(self.err_int +err*dt, -self.lim_int, self.lim_int)
        derr = (err-self.err_last)/dt
        self.err_last = err
        return self.p * err + self.i * self.err_int + self.d * derr
    
    def reset(self):
        self.err_int = 0
        self.err_last = None

class FlightController():
    def __init__(self, con):
        self.con = con
        self.telem = con.telem
        self.g_ele_con = PIDController(p_nz_ele, i_nz_ele, d_nz_ele, lim_ele_int)
        self.pit_ele_con = PIDController(p_pitch, i_pitch, 0, lim_ele_int)

        self.reset()

    def reset(self):
        self.ail = 0
        self.ele = 0
        self.rud = 0
        self.thr = 0.9

        self.q_att_sp = np.array([1, 0, 0, 0], dtype=float) # Control Setpoint now, contain roll
        self.q_att_tgt = np.array([1, 0, 0, 0], dtype=float) # Control target, no roll
        self.dir_tgt_w = np.array([1, 0, 0], dtype=float)
        self.dir_tgt_b = np.array([1, 0, 0], dtype=float)
        self.dir_sp_w = np.array([1, 0, 0], dtype=float)
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

        if self.con.OK:
            self.yaw_sp = self.telem.yaw
            self.pitch_sp = self.telem.pitch
            self.roll_sp = self.telem.roll
            print("Reset aircraft attitude setpoint")
            self.q_att_tgt = quaternion_from_euler(0, self.pitch_sp, self.yaw_sp)
            self.dir_tgt_w = q_to_dir(self.q_att_tgt)
            self.dir_sp_w = q_to_dir(self.q_att_tgt)

    def w_to_b(self, v):
        return quaternion_rotate(quaternion_inverse(self.q_att),  v)
    
    def b_to_w(self, v):
        return quaternion_rotate(self.q_att,  v)

    def set_dir_tgt(self, dir_tgt):
        self.dir_tgt_w = dir_tgt
        self.q_att_tgt = dir_to_q(self.dir_tgt_w)

    def get_dir_tgt(self):
        return self.dir_tgt_w

    def control(self, dt):
        if self.telem.tas > min_spd:
            tas_coeff  = cruise_spd*cruise_spd/(self.telem.tas*self.telem.tas)
        else:
            tas_coeff  = cruise_spd*cruise_spd/(min_spd*min_spd)

        dw = self.control_body_aim()
        self.ail = dw[0]*p_roll + p_rollrate*(self.rollrate_b_sp-self.telem.rollrate)
        ele_by_pit = self.pit_ele_con.control(dw[1], dt) 
        ele_by_rate = p_pitchrate*(self.pitchrate_b_sp-self.telem.pitchrate)
        ele_by_g = tas_coeff* self.g_ele_con.control(((-self.Nz_sp/G) - self.telem.Nz), dt)
        self.ele = ele_by_pit+ele_by_rate+ele_by_g
        print(f"""ele_by_pit {ele_by_pit*100:3.1f} ele_by_rate {ele_by_rate*100:3.1f} ele_by_g {ele_by_g*100:3.1f} ele {self.ele*100}
            (-self.Nz_sp/G) {(-self.Nz_sp/G):3.1f} self.telem.Nz {self.telem.Nz}
        """)
        self.rud = dw[2]*p_yaw + p_yawrate*(self.yawrate_b_sp-self.telem.yawrate)

    
    def set_att(self, q):
        self.q_att = q
        self.dir_now = q_to_dir(self.q_att)

    def control_body_aim(self):
        dir_tgt_w = self.dir_tgt_w
        dir_tgt_b = self.w_to_b(dir_tgt_w) 
        if dir_tgt_b[0] < MIN_X_SPHERE:
            #Then this is in aft hemisphere, so we may convert it to the nearest point at x=0 plane
            dir_tgt_b[0] = 0
            if np.linalg.norm(dir_tgt_b) > SMALL_FLOAT:
                dir_tgt_b = unit_vector(dir_tgt_b)*math.sqrt(1-MIN_X_SPHERE*MIN_X_SPHERE)
                dir_tgt_b[0] = MIN_X_SPHERE
            else:
                #Target dir is -1, 0, 0 in body frame. We turn in the right
                dir_tgt_b = np.array([math.sqrt(1-MIN_X_SPHERE*MIN_X_SPHERE), MIN_X_SPHERE, 0])
            dir_tgt_w = self.b_to_w(dir_tgt_b)
        self.dir_tgt_b = dir_tgt_b
        dv = dir_tgt_b - np.array([1, 0, 0])
        self.dv = dv = dv*self.telem.tas
        N_bz = p_dir_nz*np.dot(dv, [0, 0, 1]) # Require load project to z axis
        N_by = p_dir_nz*np.dot(dv, [0, 1, 0])  # Require load parallel to y axis
        self.N_sp_b = np.array([0, N_by, N_bz]) + self.w_to_b([0, 0, -G])
        self.Nz_sp = self.N_sp_b[2]
        self.N_sp_w = self.b_to_w(-self.N_sp_b)

        N_sp_w_v = unit_vector(self.N_sp_w)
        #We should set up the new q_att here by N_sp_w and new vel which vertical to N_sp_w but at same plane with dir now
        self.dir_sp_w = unit_vector(self.dir_now + self.b_to_w(np.array([0, N_by, N_bz]))/self.telem.tas) #What happen when N_sp_w and dir now is parallel?

        self.q_att_sp = dir_to_q(dir_tgt_w, N_sp_w_v)

        self.roll_sp, self.pitch_sp, self.yaw_sp = euler_from_quaternion(self.q_att_sp)
        print("dir_tgt_w", dir_tgt_w, "dir_tgt_b", dir_tgt_b)
        dw = att_err_to_tangent_space(self.q_att_sp, self.q_att)
        
        self.dw_sp = dw
        return dw
