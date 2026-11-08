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
rov_center_of_mass = [0.002023, -0.133479, 0.01563]
imu_position = [0, -0.25, 0.042]

rov_ip = '172.25.250.1'

# mass, kg
rov_mass = 8.17

# name: human-readable name
# location: position in ROV's coordinate system, units meters
# orientation: unit vector representing forward (round, not pointy) direction of thruster
# pin: raspberry pi pin on which thruster is connected
# model: 't-100' or 't-200' depending on which Blue Robotics thruster it is (different thrust characteristics)
# direction: corrects for ESC wiring if thruster runs in the wrong direction.
# handing: CW thruster prop (default) is 1, CCW is -1
    # purple = -1, blue = 1 (tape)
# thruster locations just given in terms of CAD labelling (might need to reswitch later assuming they are the same)
# thruster names are also probably wildly inaccurate

# TODO: Update this everytime ROV is put togehter again and ESCs are moved
_THRUSTER_FIELDS = ('name', 'location', 'orientation', 'pin', 'model', 'direction', 'handing', 'letter', 'slot')
thruster_config = [dict(zip(_THRUSTER_FIELDS, row)) for row in [
    # name         location                           orientation       pin  model   dir hand letter slot
    ('top',        [0.005252, -0.118399, 0.158790],   [1.0, 0.0, 0.0],  26, 't-200',  1, -1, 'A', 6),
    ('bottom',     [0.074161, -0.118399, -0.158790],  [1.0, 0.0, 0.0],  13, 't-200',  1,  1, 'D', 2),
    ('left_up',    [-0.153494, -0.118504, 0.077573],  [0.0, 0.0, 1.0],  12, 't-200',  1, -1, 'E', 4),
    ('right_up',   [0.153494, -0.118292, 0.0775735],  [0.0, 0.0, 1.0],  16, 't-200', -1, -1, 'F', 5),
    ('left_back',  [-0.153494, -0.021175, 0.000105],  [0.0, -1.0, 0.0], 25, 't-200', -1, -1, 'C', 3),
    ('right_back', [0.153494, -0.021175, -0.000139],  [0.0, -1.0, 0.0],  9, 't-200',  1, -1, 'B', 2),
]]

_MOTOR_FIELDS = ('name', 'pin', 'model', 'direction', 'slot')
motor_config = [dict(zip(_MOTOR_FIELDS, row)) for row in [
    # name            pin model   dir slot
    ('buoyancy_arm',  19, 'm_200', 1, 7),  # fwd right 19
    ('gantry_right',   6, 'm_200', 1, 9),
    ('gantry_left',   13, 'm_200', 1, 8),
    ('manipulator',   20, 'm_200', 1, 1),
]]
