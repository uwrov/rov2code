import pathlib
import subprocess
import websockets
import asyncio
import json
import cv2
import threading
import time

# import numpy as np

sources = [
            "http://172.25.250.1:8554/",
            "http://172.25.250.1:8555/",
            "http://172.25.250.1:8556/",
        ]
window_names = ["Birdseye", "Rear", "Front"]
positions = [
    (900, 0),
    (900, 400),
    (0, 160)
]
sizes = [
    (630, 400),
    (630, 540),
    (900, 700)
]
orientations = [0, 270,0]
class LatestFrameCapture:
    def __init__(self, source):
        self.source = source
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        self.cap = None

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _open(self):
        self.cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def _capture_loop(self):
        while self.running:
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

    def close(self):
        self.running = False

        if self.cap is not None:
            self.cap.release()

class Interface():
    def __init__(self, _core: 'Core'):
        self.core = _core
        self.task = None
        self.websocket = None
        for name, position, size, orientation in zip(window_names, positions, sizes, orientations):
            cv2.namedWindow(name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.moveWindow(name, *position)
            cv2.resizeWindow(name, *size)

        self.captures = [LatestFrameCapture(source) for source in sources]
        uri = 'ws://localhost:8002'
        cwd = (pathlib.Path(__file__).parent / 'godot').resolve()
        if False:
            subprocess.Popen(['godot', 'interface.tscn', '-u', uri], cwd=cwd)
        else:
            subprocess.Popen(['godot', 'interface.tscn', '-u', uri], cwd=cwd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def set_task(self, task: 'Task'):
        self.task = task

    def notify_new_sensor_data(self):
        print(f'accelerometer: {self.core.accelerometer}, gyro: {self.core.gyroscope}, thrusters: {self.core.thrusters}, motors: {self.core.motors}')

    async def server_handler(self, _websocket):
        self.websocket = _websocket
        async for message in self.websocket:
            data = json.loads(str(message)[2:-1])
            result = json.dumps(data)
            await self.core.consume_interface_websocket(data)

    async def notify_sensor_update(self):
        if self.websocket != None:
            await self.websocket.send(json.dumps({
                'accelerometer': self.core.accelerometer,
                'quaternion': self.core.quaternion,
                'thrusters': self.core.thrusters,
                'motors': self.core.motors,
                'gantry' : self.core.gantry,
                'arm_angle' : self.core.arm_angle
            }))
    def update_video_streams(self):
        for capture, window_name, orientation in zip(self.captures, window_names, orientations):
            frame = capture.get()

            if frame is not None:
                if orientation == 0:
                    cv2.imshow(window_name, frame)
                elif orientation == 90:
                    cv2.imshow(window_name, cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE))
                elif orientation == 270:
                    cv2.imshow(window_name, cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE))
                else:
                    cv2.imshow(window_name, cv2.rotate(frame, cv2.ROTATE_180))

        cv2.waitKey(1)
    async def video_loop(self):
        while True:
            self.update_video_streams()
            await asyncio.sleep(1 / 60)

