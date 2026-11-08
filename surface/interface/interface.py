import pathlib
import subprocess
import asyncio
import json
import cv2
import threading
import time
from typing import NamedTuple

class Camera(NamedTuple):
    port: int
    window: str
    position: tuple
    size: tuple
    rotation: int  # degrees clockwise: 0, 90, 180 or 270

CAMERAS = [  # update rotation as needed
    Camera(8554, "Birdseye", (900, 0),   (630, 400), 0),
    Camera(8555, "Bottom",   (900, 400), (630, 540), 270),
    Camera(8556, "Front",    (0, 160),   (900, 700), 0),
]

ROTATIONS = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}

SENSOR_UPDATE_FIELDS = ('accelerometer', 'quaternion', 'thrusters', 'motors', 'gantry', 'arm_angle')

class LatestFrameCapture:
    def __init__(self, source):
        self.source = source
        self.frame = None
        self.lock = threading.Lock()
        self.cap = None

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _open(self):
        self.cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def _capture_loop(self):
        while True:
            if self.cap is None or not self.cap.isOpened():
                self._open()

                if not self.cap.isOpened():
                    time.sleep(1)
                    continue

            success, frame = self.cap.read()

            if success:
                with self.lock:
                    self.frame = frame
            else:
                self.cap.release()
                self.cap = None
                time.sleep(0.25)

    def get(self):
        with self.lock:
            return self.frame

class Interface():
    def __init__(self, _core: 'Core'):
        self.core = _core
        self.websocket = None
        for camera in CAMERAS:
            cv2.namedWindow(camera.window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.moveWindow(camera.window, *camera.position)
            cv2.resizeWindow(camera.window, *camera.size)

        self.captures = [LatestFrameCapture(f"http://172.25.250.1:{camera.port}/") for camera in CAMERAS]
        uri = 'ws://localhost:8002'
        cwd = (pathlib.Path(__file__).parent / 'godot').resolve()
        subprocess.Popen(['godot', 'interface.tscn', '-u', uri], cwd=cwd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    async def server_handler(self, _websocket):
        self.websocket = _websocket
        async for message in self.websocket:
            data = json.loads(message)
            await self.core.consume_interface_websocket(data)

    async def notify_sensor_update(self):
        if self.websocket is not None:
            await self.websocket.send(json.dumps({
                name: getattr(self.core, name) for name in SENSOR_UPDATE_FIELDS
            }))

    def update_video_streams(self):
        for capture, camera in zip(self.captures, CAMERAS):
            frame = capture.get()

            if frame is not None:
                if camera.rotation:
                    frame = cv2.rotate(frame, ROTATIONS[camera.rotation])
                cv2.imshow(camera.window, frame)

        cv2.waitKey(1)

    async def video_loop(self):
        while True:
            self.update_video_streams()
            await asyncio.sleep(1 / 60)

