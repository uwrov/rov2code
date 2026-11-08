import numpy as np
from .rov_config import thruster_config
from .rov_config import rov_center_of_mass

# looks like a reasonable max on motor power is 3.87757526

#direction corresponds to ESC wiring. When thruster receives high PWM (>1500), clockwise spin when viewed from the back is defined as direction 1


orientations = np.array([t['orientation'] for t in thruster_config])
displacements = np.array([t['location'] for t in thruster_config]) - rov_center_of_mass

# each column is one thruster's force (x, y, z) followed by its torque (x, y, z)
control_mat = np.vstack([orientations.T, np.cross(displacements, orientations).T])

# Conduit cannot roll
control_mat_5dof = np.delete(control_mat, 3, axis=0)
control_inv = np.insert(np.linalg.pinv(control_mat_5dof), 3, 0, axis=1)

def convert_force_and_torque_to_motor_powers(vector) -> np.ndarray:
    return control_inv @ np.asarray(vector)
