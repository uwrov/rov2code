import numpy as np

from .state import VehicleState
from ..frontend.orb_frontend import ORBFrontend
from ..sensors.imu_propagation import IMUPropagation
from ..sensors.depth_constraint import DepthConstraint
from ..backend.local_mapping import LocalMapping


class SLAMSystem:

    def __init__(self, K):

        self.K = K
        self.state = VehicleState()
        self.frontend = ORBFrontend(K)
        self.imu = IMUPropagation()
        self.depth = DepthConstraint()
        self.mapper = LocalMapping(K)
        self.trajectory = []
        self.initialized = False

        print("SLAM System Initialized")


    def step(self, frame, gyro, accel, depth_measurement, dt):

        self.state = self.imu.propagate(
            self.state,
            gyro,
            accel,
            dt
        )

        visual_result = self.frontend.process(frame)

        if visual_result is not None:

            R_rel, t_rel = visual_result
            t_dir = t_rel.flatten()

            if np.linalg.norm(t_dir) > 1e-6:
                t_dir /= np.linalg.norm(t_dir)
            else:
                t_dir = np.zeros(3)

            step_size = 0.05
            self.state.position += (
                self.state.orientation @ (t_dir * step_size)
            )

            self.state.orientation = (
                self.state.orientation @ R_rel
            )

            kp = self.frontend.prev_kp
            desc = self.frontend.prev_desc

            if kp is not None and desc is not None:

                self.mapper.maybe_add_keyframe(
                    frame,
                    kp,
                    desc,
                    self.state.orientation,
                    self.state.position.reshape(3, 1)
                )

                self.mapper.prune_map()

        self.state = self.depth.apply(
            self.state,
            depth_measurement
        )

        self.trajectory.append(
            self.state.position.copy()
        )

        return self.state


    def get_current_pose(self):

        return (
            self.state.position.copy(),
            self.state.orientation.copy()
        )


    def get_trajectory(self):

        return np.array(self.trajectory)


    def get_map_points(self):

        if len(self.mapper.map_points) == 0:
            return np.empty((0, 3))

        return np.array(self.mapper.map_points)