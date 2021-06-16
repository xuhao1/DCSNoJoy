import numpy as np
# control_style = "battlefield" 
control_style = "warthunder" 

#P51
#CAMERA_X = 20
#CAMERA_Z = 4

ailrate = 1.5
elerate = 2.0
THR_RATE = 0.7

#Rate of free view and mouse aim
att_sp_rate = 0.5

# #For F18
p_dir_nz = 0.3
p_roll = 2.0
p_pitch = 4.0
i_pitch = 0.1
lim_ele_int = 0.3
p_yaw = 4.0 

p_rollrate = 1.0
p_pitchrate = 0.4
p_yawrate = 1.0

p_nz_ele = 0.02
i_nz_ele = 0.1
d_nz_ele = 0.0

cruise_spd = 300 #cruise_spd for pid, in m/s
min_spd = 100
CAMERA_X = 30
CAMERA_Z = 5

#For F51
# p_dir_nz = 0.08
# p_roll = 3.0
# p_pitch = 8.0
# p_yaw = 10.0 
# p_rollrate = 1.0
# p_pitchrate = 0.4
# p_yawrate = 1.0
# p_nz_ele = 0.1
# cruise_spd = 200 #cruise_spd for pid, in m/s
# min_spd = 80
# CAMERA_X = 20
# CAMERA_Z = 5

# For VIEW
view_rate = 2.0
view_filter_rate = 0.005

CAM_PITCH_OFFSET = -10/57.3
#Keyboard shortcuts
keyboard_freelook = "c"

keyboard_inc_thr = "shift"
keyboard_dec_thr = "ctrl"

keyboard_ele_max = "s"
keyboard_ele_min = "w"

keyboard_ail_max = "d"
keyboard_ail_min = "a"

keyboard_rud_max = "e"
keyboard_rud_min = "q"

keyboard_exit = "shift+esc"

# Do not modify below unless you understand
USE_VJOY = False
USE_OPENTRACK = False
MAIN_WIN_DURATION = 5
DCS_TIMEOUT = 0.1
HIDE_WIHTOUT_DCS = False

#Only works in external view
ACTIVE_CTRL_VIEW = True

ACTIVE_CTRL_F3 = False
G = 9.8
# DEFAULT_FOV = 85/57.3
DEFAULT_FOV = 85/57.3

UDP_IP = "127.0.0.1"
UDP_PORT = 4242

DCS_UDP_IP = "127.0.0.1"
DCS_UDP_PORT = 27015
DCS_UDP_SEND_PORT = 27016

R_NUEtoNED = np.array([
    [1, 0, 0],
    [0, 0, 1],
    [0, -1, 0]
])
