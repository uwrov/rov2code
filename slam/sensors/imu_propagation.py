import numpy as np
from scipy.spatial.transform import Rotation

class IMUPropagation:

    def __init__(self):
        self.gravity = np.array([0, 0, -9.81])

    def propagate(self, state, gyro, accel, dt):

        rotvec = gyro * dt
        dR = Rotation.from_rotvec(rotvec).as_matrix()
        state.orientation = state.orientation @ dR
        accel_world = state.orientation @ accel - self.gravity
        state.velocity += accel_world * dt
        state.position += state.velocity * dt

        return state