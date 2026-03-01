import numpy as np
import cv2


class Keyframe:

    def __init__(self, frame, keypoints, descriptors, R, t):
        self.frame = frame
        self.keypoints = keypoints
        self.descriptors = descriptors
        self.R = R.copy()
        self.t = t.copy()


class LocalMapping:

    def __init__(self, K):

        self.K = K
        self.keyframes = []
        self.map_points = []
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.keyframe_distance_threshold = 0.25

        self.last_keyframe_pose = None

    def maybe_add_keyframe(self, frame, kp, desc, R, t):

        if self.last_keyframe_pose is None:
            self._add_keyframe(frame, kp, desc, R, t)
            return

        prev_R, prev_t = self.last_keyframe_pose
        dist = np.linalg.norm(t - prev_t)

        if dist > self.keyframe_distance_threshold:
            self._add_keyframe(frame, kp, desc, R, t)
            self._triangulate_last_two()


    def _add_keyframe(self, frame, kp, desc, R, t):

        kf = Keyframe(frame, kp, desc, R, t)
        self.keyframes.append(kf)
        self.last_keyframe_pose = (R.copy(), t.copy())


    def _triangulate_last_two(self):

        if len(self.keyframes) < 2:
            return

        kf1 = self.keyframes[-2]
        kf2 = self.keyframes[-1]

        matches = self.matcher.knnMatch(
            kf1.descriptors,
            kf2.descriptors,
            k=2
        )

        good = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good.append(m)

        if len(good) < 30:
            return

        pts1 = np.float32(
            [kf1.keypoints[m.queryIdx].pt for m in good]
        )
        pts2 = np.float32(
            [kf2.keypoints[m.trainIdx].pt for m in good]
        )

        P1 = self.K @ np.hstack((kf1.R, kf1.t))
        P2 = self.K @ np.hstack((kf2.R, kf2.t))

        pts4d = cv2.triangulatePoints(
            P1, P2,
            pts1.T,
            pts2.T
        )

        pts3d = pts4d[:3] / pts4d[3]
        valid = pts3d[2] > 0
        pts3d = pts3d[:, valid]

        for i in range(pts3d.shape[1]):
            point = pts3d[:, i]
            self.map_points.append(point)


    def prune_map(self):

        if len(self.map_points) == 0:
            return

        pts = np.array(self.map_points)
        mean = np.mean(pts, axis=0)
        dist = np.linalg.norm(pts - mean, axis=1)

        keep = dist < 5.0

        self.map_points = pts[keep].tolist()