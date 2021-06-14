# DCS cooridnate system
north-up-east cooridnate system
x: north y: height z: east

General aviation (x', y', z')
north-east-down

DCS to Aviation
R_NUEtoNED = [
    [1, 0, 0],
    [0, 0, 1],
    [0, -1, 0]
]

For camera
Rotation:
R = eye(3)
The view is point to north, left screen is west, right screen is east up down correct

euler_matrix(0, 0, np.pi/2)
    Rotate yaw + 90, the view is watch ground

euler_matrix(0, 0, -np.pi/2)
    Rotate yaw - 90, the view is watch sky

euler_matrix(pi/2, 0, 0) or euler_matrix(-pi/2, 0, 0)
    Watch north, no different with identity

euler_matrix(0, pi/2, 0)
    Watch east, up down correct, left screen is north right is south

euler_matrix(0, -pi/2, 0)
    Watch west, up down correct, left screen is north right is south

So naive way is
Rcam_dcs = euler_matrix(0, view_yaw, -view_pitch)

Translation:
T = aircraft_pos + [0, 0, 0] # camera is near center of aircraft
T = aircraft_pos + [30, 0, 0] # camera is at north of aircraft
T = aircraft_pos + [0, 30, 0] # camera is at upward the aircraft
T = aircraft_pos + [0, -30, 0] # camera i at downward of aircraft

T = aircraft_pos + [0, 0, 30] # camera is at east the aircraft
T = aircraft_pos + [0, 0, -30] # camera i at west of aircraft

To convert cam pos to make it soround the aircraft

Tcam_ned = Rcam_ned*Toffset_ned
Tcam_dcs[1] = -Tcam_ned[2]
Tcam_dcs[2] = Tcam_ned[1]