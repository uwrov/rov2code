import numpy as np
from collections import deque

class PoseFilter:
    def __init__(self, buffer_size: int = 50):
        self.history = deque(maxlen=buffer_size)

    def update(self, new_pose: np.ndarray) -> np.ndarray:
        self.history.append(new_pose)
        return np.mean(self.history, axis=0)