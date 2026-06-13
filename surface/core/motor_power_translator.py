import numpy as np
from .rov_config import thruster_config
from .rov_config import rov_center_of_mass

# from geometry_msgs.msg import Wrench

# thruster order: ['forward_left', 'forward_right', 'forward_top', 'sideways_top', 'up_left', 'up_right']
# looks like a reasonable max on motor power is 3.87757526

#direction corresponds to ESC wiring. When thruster receives high PWM (>1500), clockwise spin when viewed from the back is defined as direction 1


control_mat = np.zeros((6, 6))

# first three rows of matrix are x, y, z force contributions
for i in range(6):
    control_mat[0:3, i] = thruster_config[i]['orientation']

# last three rows of matrix are x, y, z torque vector
for i in range(6):
    displacement = np.subtract(thruster_config[i]['location'], rov_center_of_mass)
    force = thruster_config[i]['orientation']
    control_mat[3:6, i] = np.cross(displacement, force)

control_mat_5dof = np.delete(control_mat, 3, axis=0)

control_inv = np.linalg.pinv(control_mat_5dof)
control_inv = np.insert(control_inv, 3, 0, axis=1)

def convert_force_and_torque_to_motor_powers(vector) -> np.array:
    input_vector = np.array([vector]).T
    motor_powers = control_inv @ input_vector
    return motor_powers
