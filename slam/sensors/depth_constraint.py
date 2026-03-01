class DepthConstraint:

    def apply(self, state, depth_measurement):
        state.position[2] = -depth_measurement
        return state