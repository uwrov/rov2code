import numpy as np

class VehicleState:

    def __init__(self):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.eye(3)

    def copy(self):
        s = VehicleState()
        s.position = self.position.copy()
        s.velocity = self.velocity.copy()
        s.orientation = self.orientation.copy()
        return s