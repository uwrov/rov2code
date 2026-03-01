import cv2
import numpy as np

class ORBFrontend:

    def __init__(self, K):
        self.K = K
        self.detector = cv2.ORB_create(2000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.prev_kp = None
        self.prev_desc = None
        self.prev_frame = None

    def process(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, desc = self.detector.detectAndCompute(gray, None)

        if self.prev_frame is None:
            self.prev_frame = gray
            self.prev_kp = kp
            self.prev_desc = desc
            return None

        matches = self.matcher.knnMatch(self.prev_desc, desc, k=2)

        good = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good.append(m)

        if len(good) < 40:
            return None

        pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in good])
        pts2 = np.float32([kp[m.trainIdx].pt for m in good])

        E, mask = cv2.findEssentialMat(
            pts1, pts2, self.K,
            method=cv2.RANSAC, prob=0.999, threshold=1.0
        )

        if E is None:
            return None

        _, R, t, mask = cv2.recoverPose(E, pts1, pts2, self.K)

        self.prev_frame = gray
        self.prev_kp = kp
        self.prev_desc = desc

        return R, t