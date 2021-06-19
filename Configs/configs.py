import numpy as np
# control_style = "battlefield" 
control_style = "warthunder" 
default_config_file = "default"

# mouse_joystick_elemode = "aoa"
# mouse_joystick_elemode = "pitchrate"
mouse_joystick_elemode = "gcmd"
#P51
#CAMERA_X = 20
#CAMERA_Z = 4

ailrate = 2.0
elerate = 1.0
grate = 30.0
aoarate = 0.5
THR_RATE = 0.7

#Rate of free view and mouse aim
att_sp_rate = 0.5

# For VIEW
view_rate = 2.0
view_filter_rate = 0.008
view_filter_rate_freelook = 0.05
filter_rate_of_filter_rate = 0.05

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


VMOUSE_SIZE = 100