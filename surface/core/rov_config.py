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
# this is currently being calculated without knowing materials in CAD
rov_center_of_mass = [
    0.002023,
    -0.133479,
    0.01563
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
# orientation: unit vector representing forward (round, not pointy) direction of thruster
# pin: raspberry pi pin on which thruster is connected
# model: 't-100' or 't-200' depending on which Blue Robotics thruster it is (different thrust characteristics)
# handing: CW thruster prop (default) is 1, CCW is -1
    # purple = -1, blue = 1 (tape)
# direction: corrects for ESC wiring if thruster runs in the wrong direction.
# thruster locations just given in terms of CAD labelling (might need to reswitch later assuming they are the same)
# thruster names are also probably wildly inaccurate

#PIN IS FIRST
# TODO: Update this everytime ROV is put togehter again and ESCs are moved
# pin_slot_map = {
#     6 :
#     9 : 
#     11 :
#     12 : 
#     13 : ,
#     16 :
#     19 : 
#     20 : ,
#     25 : ,
#     26 : 

thruster_config = [
    {
        'name': 'top',
        'location': [0.005252, -0.118399, 0.158790],
        'orientation': [1.0, 0.0, 0.0],
        'pin': 26, 
        'model': 't-200',
        'direction': 1,
        'handing' : -1,
        'letter': 'A',
        'slot' : 6,
    },
    {
        'name': 'bottom',
        'location': [0.074161, -0.118399, -0.158790],
        'orientation': [1.0, 0.0, 0.0],
        'pin': 13,
        'model': 't-200',
        'direction': 1,
        'handing' : 1,
        'letter': 'D',
        'slot' : 2,
    },
    {
        'name': 'left_up',
        'location': [-0.153494, -0.118504, 0.077573],
        'orientation': [0.0, 0.0, 1.0],
        'pin': 12,
        'model': 't-200',
        'direction': 1,
        'handing' : -1,
        'letter': 'E',
        'slot' : 4,
    },
    {
        'name': 'right_up',
        'location': [0.153494, -0.118292, 0.0775735],
        'orientation': [0.0, 0.0, 1.0],
        'pin': 16,
        'model': 't-200',
        'direction': -1,
        'handing' : -1,
        'letter': 'F',
        'slot' : 5,
    },
    {
        'name': 'left_back',
        'location': [-0.153494, -0.021175, 0.000105],
        'orientation': [0.0, -1.0, 0.0],
        'pin': 25,
        'model': 't-200',
        'direction': -1,
        'handing' : -1,
        'letter': 'C',
        'slot' : 3,
    },
    {
        'name': 'right_back',
        'location': [0.153494, -0.021175, -0.000139],
        'orientation': [0.0, -1.0, 0.0],
        'pin': 9,
        'model': 't-200',
        'direction': 1 ,
        'handing' : -1,
        'letter': 'B',
        'slot' : 2,
    },
]
motor_config = [
    {
        'name': 'buoyancy_arm',
        # 'location': [0.0, 0.0, 0.0],
        # 'orientation': [0.0, 1.0, 0.0],
        'pin': 19, #fwd right 19
        'model': 'm_200',
        'direction': 1,
        'slot' : 7,
    },
    {
        'name': 'gantry_right',
        # 'location': [0.0, 0.0, 0.0],
        # 'orientation': [0.0, 1.0, 0.0],
        'pin': 6,
        'model': 'm_200',
        'direction': 1,
        'slot' : 9,
    },
    {
        'name': 'gantry_left',
        # 'location': [0.0, 0.0, 0.0],
        # 'orientation': [0.0, 1.0, 0.0],
        'pin': 13, 
        'model': 'm_200',
        'direction': 1,
        'slot' : 8,
    },
    {
        'name': 'manipulator',
        # 'location': [0.0, 0.0, 0.0],
        # 'orientation': [0.0, 1.0, 0.0],
        'pin': 20,
        'model': 'm_200',
        'direction': 1,
        'slot' : 1,
    }
]
