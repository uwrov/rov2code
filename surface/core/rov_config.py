# coordinate system is Onshape coordinates, i.e.:
# 'forward'/front is towards -y, 'up'/top is towards +z, and the 'right' side of
# the ROV (i.e. to the left of its forward path of motion) is +x

# pin configuration guessed by Alnis (@alnis#0001) on 2023-01-22 from old code:
# https://github.com/uwrov/nautilus_pi/blob/main/src/uwrov_auto/scripts/motor_driver.py

# it's possible that something is left-right mirrored...

# thruster locations and center of mass of ROV retrieved by rowan @romainne
# on 2025-05-03 from
# https://cad.onshape.com/documents/9c4723f7c69c6ee6cd4e6801/v/24d1f3e58de7b40f5f2fb6c2/e/c7bd78bf5f17a1d0aef05f66 

#values in meters
rov_center_of_mass = [
    0.002,
    # -0.178,
    -.158,
    0.007
]
imu_position  = [
    0,
    -0.25,
    0.042,
]
# rov_center_of_mass = [
#     0.01,
#     -0.155121,
#     0.00018
# ]

#  ROV mass moments of inertia
#[[Lxx, Lyx, Lzx][Lxy,Lyy,Lzy][Lxz,Lyz,Lzz]],kg*m^2

# rov_mass_moments_of_inertia =[[0.139,6.754*10^-5,1.411*10^-5],[6.754*10^-5,0.085,-0.009][1.411*10^-5,-0.009,0.141]]

# mass, kg
rov_mass=8.17

# name: human-readable name
# location: position in ROV's coordinate system, units meters
# orientation: unit vector representing forward direction of thruster
# pin: raspberry pi pin on which thruster is connected
# model: 't-#' (new name, forgot to write down what it was)
# handing: CW thruster prop (default) is 1, CCW is -1
# direction: corrects for ESC wiring if thruster runs in the wrong direction.

# thruster locations just given in terms of CAD labelling (might need to reswitch later assuming they are the same)
thruster_config = [
    {
        'name': 'forward_left',
        # 'location': [0.13354, -0.28541, -0.043956],
        'location': [0.008588, 0.012029, 0.201448],
        'orientation': [0.0, -1.0, 0.0],
        'pin': 19,
        'model': 't-#',
        'direction': -1,
        'handing' : 1,
        'letter': 'A',
    },
    {
        'name': 'forward_right',
        'location': [0.079666, 0.011938, -0.105854],
        'orientation': [0.0, -1.0, 0.0],
        'pin': 12,
        'model': 't-#',
        'direction': -1,
        'handing' : 1,
        'letter': 'D',
    },
    {
        'name': 'forward_top',
        'location': [0.154729, 0.011938, 0.129965],
        'orientation': [0.0, -1.0, 0.0],
        'pin': 16,
        'model': 't-#',
        'direction': 1,
        'handing' : -1,
        'letter': 'E',
    },
    {
        'name': 'sideways_top', #4
        # 'location': [-0.001,-0.222,0.14],
        'location': [-0.152573,0.014938, 0.869965],
        'orientation': [1.0, 0.0, 0.0],
        'pin': 10,
        'model': 't-#',
        'direction': 1,
        'handing' : 1,
        'letter': 'F',
    },
    {
        'name': 'up_left', #5
        'location': [0.154729, 0.109131, 0.047902],
        'orientation': [0.0, 0.0, 1.0],
        'pin': 25,
        'model': 't-#',
        'direction': -1,
        'handing' : 1,
        'letter': 'C',
    },
    {
        'name': 'up_right',
        'location': [-0.152573,0.109131,0.047902],
        'orientation': [0.0, 0.0, 1.0],
        'pin': 26,
        'model': 't-#',
        'direction': -1 ,
        'handing' : -1,
        'letter': 'B',
    },
]