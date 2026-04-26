#!/usr/bin/env python3
"""
AprilTag pose → gantry motion: each tag ID maps to a desired tag pose in the camera
frame; proportional control jogs the gantry to reduce error.

Tune CAMERA_PARAMS with a real calibration for trustworthy poses. Defaults are a rough
640×480 guess. Use DebugGantryDriver (default) to verify logic without hardware; swap
in your step/dir or serial driver by subclassing GantryDriver.
"""

from __future__ import annotations

import argparse
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple

import cv2
import numpy as np
from pupil_apriltags import Detector

from PoseFilter import PoseFilter

# --- Camera & tag (meters for pose; fx, fy, cx, cy in pixels) ---
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
# Rough placeholder intrinsics ~60° HFOV at 640px — replace with cv2.calibrateCamera output.
FX = 550.0
FY = 550.0
CX = FRAME_WIDTH / 2.0
CY = FRAME_HEIGHT / 2.0
TAG_SIZE_M = 0.05  # Outer black square edge length of the printed tag

# Desired tag translation in the camera frame (meters). AprilTag library: tag center
# in front of the camera; +Z usually along optical axis (check your setup / drawing).
TAG_TARGETS: Dict[int, np.ndarray] = {
    1: np.array([0.0, 0.0, 0.35], dtype=np.float64),
    2: np.array([0.0, 0.0, 0.25], dtype=np.float64),
}
# If several mapped tags are visible, the first listed ID here wins.
TAG_PRIORITY: List[int] = [1, 2]

# Control: error (m) → jog (mm) this frame, then clipped. Map camera errors into gantry axes.
KP_MM_PER_M = np.array([80.0, 80.0, 40.0], dtype=np.float64)  # x, y, z
MAX_STEP_MM = 3.0
DEADBAND_M = 0.003
ENABLE_Z = False
# Rows: gantry (gx, gy, gz); cols: camera-frame error (ex, ey, ez). Identity = 1:1.
AXIS_MAP = np.eye(3, dtype=np.float64)

DETECTOR_KWARGS = dict(families="tag36h11", quad_decimate=2.0, refine_edges=1)


@dataclass
class GantryState:
    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0


class GantryDriver(ABC):
    """Send incremental motion (mm) along gantry axes for one control tick."""

    @abstractmethod
    def jog_mm(self, gx: float, gy: float, gz: float) -> None:
        pass


class DebugGantryDriver(GantryDriver):
    """Accumulates commanded motion and prints (for Mac / bench test without motors)."""

    def __init__(self) -> None:
        self.state = GantryState()
        self.verbose = True

    def jog_mm(self, gx: float, gy: float, gz: float) -> None:
        self.state.x_mm += gx
        self.state.y_mm += gy
        self.state.z_mm += gz
        if self.verbose and (abs(gx) + abs(gy) + abs(gz)) > 1e-6:
            print(
                f"  jog_mm gx={gx:+.3f} gy={gy:+.3f} gz={gz:+.3f}  "
                f"→ pos ({self.state.x_mm:.2f}, {self.state.y_mm:.2f}, {self.state.z_mm:.2f})"
            )


class StepDirGantryDriver(GantryDriver):
    """
    Example hook: convert mm to steps and toggle step/dir GPIOs.
    Requires pigpio on the Pi and per-axis pins/steps-per-mm filled in.
    """

    def __init__(self, steps_per_mm: Tuple[float, float, float]) -> None:
        self.steps_per_mm = np.array(steps_per_mm, dtype=np.float64)
        try:
            import pigpio  # type: ignore
        except ImportError as e:
            raise RuntimeError("StepDirGantryDriver needs pigpio (Raspberry Pi).") from e
        self._pigpio = pigpio
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("pigpio daemon not running (sudo pigpiod).")
        # TODO: set_mode for STEP/DIR pins; store pin numbers per axis.
        self._pins_ready = False

    def jog_mm(self, gx: float, gy: float, gz: float) -> None:
        if not self._pins_ready:
            raise RuntimeError("Configure STEP/DIR pins and set _pins_ready in StepDirGantryDriver.")
        deltas = np.array([gx, gy, gz], dtype=np.float64)
        steps = np.rint(deltas * self.steps_per_mm).astype(int)
        # TODO: pulse STEP lines according to sign on DIR for each axis.
        _ = steps


def camera_params() -> Tuple[float, float, float, float]:
    return (FX, FY, CX, CY)


def pick_detection(
    results: Iterable, targets: Dict[int, np.ndarray], priority: List[int]
):
    by_id = {r.tag_id: r for r in results}
    for tid in priority:
        if tid in by_id and tid in targets:
            return by_id[tid], tid
    return None, None


def pose_translation_m(det) -> np.ndarray:
    t = np.asarray(det.pose_t, dtype=np.float64).reshape(3)
    return t


def control_delta_mm(
    target_m: np.ndarray, actual_m: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    err_cam = target_m - actual_m
    if np.linalg.norm(err_cam) < DEADBAND_M:
        return np.zeros(3), err_cam
    jog_cam_mm = KP_MM_PER_M * err_cam * 1000.0
    if not ENABLE_Z:
        jog_cam_mm[2] = 0.0
    gantry_mm = AXIS_MAP @ jog_cam_mm
    gantry_mm = np.clip(gantry_mm, -MAX_STEP_MM, MAX_STEP_MM)
    return gantry_mm, err_cam


def draw_overlay(
    frame,
    det,
    tag_id: int,
    err_m: np.ndarray,
    driver: GantryDriver,
) -> None:
    (ptA, ptB, ptC, ptD) = det.corners
    pts = [tuple(p.astype(int)) for p in (ptA, ptB, ptC, ptD, ptA)]
    for a, b in zip(pts, pts[1:]):
        cv2.line(frame, a, b, (0, 200, 255), 2)
    t = pose_translation_m(det)
    msg = f"id={tag_id} t=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})m"
    cv2.putText(frame, msg, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    em = f"err=({err_m[0]:+.4f},{err_m[1]:+.4f},{err_m[2]:+.4f})m"
    cv2.putText(frame, em, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
    if isinstance(driver, DebugGantryDriver):
        s = driver.state
        pos = f"sim gantry mm: ({s.x_mm:.1f}, {s.y_mm:.1f}, {s.z_mm:.1f})"
        cv2.putText(frame, pos, (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 255), 1)


def main() -> None:
    p = argparse.ArgumentParser(description="AprilTag ID → gantry pose hold (visual servo).")
    p.add_argument("--camera", type=int, default=CAMERA_INDEX)
    p.add_argument("--no-window", action="store_true", help="No OpenCV window (headless).")
    p.add_argument("--quiet", action="store_true", help="Less console output for DebugGantryDriver.")
    args = p.parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        raise SystemExit("Could not open camera.")

    detector = Detector(**DETECTOR_KWARGS)
    cam = camera_params()
    driver: GantryDriver = DebugGantryDriver()
    if isinstance(driver, DebugGantryDriver):
        driver.verbose = not args.quiet

    print("AprilTag gantry nav — 'q' quit. Mapped IDs:", list(TAG_TARGETS.keys()))
    print("Calibrate CAMERA_PARAMS and TAG_SIZE_M for real robots.")

    # Then in main():
    filter = PoseFilter(buffer_size=10)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=cam,
            tag_size=TAG_SIZE_M,
        )
        det, tid = pick_detection(results, TAG_TARGETS, TAG_PRIORITY)

        if det is not None:
            actual = filter.update(pose_translation_m(det))

        if det is not None and tid is not None:
            target = TAG_TARGETS[tid]
            actual = pose_translation_m(det)
            gantry_mm, err_m = control_delta_mm(target, actual)
            driver.jog_mm(float(gantry_mm[0]), float(gantry_mm[1]), float(gantry_mm[2]))
            if not args.no_window:
                draw_overlay(frame, det, tid, err_m, driver)
        else:
            if not args.no_window:
                cv2.putText(
                    frame,
                    "No mapped tag in view",
                    (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (100, 100, 255),
                    2,
                )

        if not args.no_window:
            cv2.imshow("gantry_apriltag_nav", frame)
            if cv2.waitKey(1) == ord("q"):
                break

    cap.release()
    if not args.no_window:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
