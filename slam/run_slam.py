import asyncio
import json
import numpy as np
import cv2
import time
import websockets

from core.slam_system import SLAMSystem


async def slam_client():

    uri = "ws://localhost:8001"

    #Get the real address
    cap = cv2.VideoCapture("http://172.25.250.1:8554/")

    fx = fy = 600
    cx = 320
    cy = 240

    K = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])

    slam = SLAMSystem(K)

    prev_time = time.time()

    async with websockets.connect(uri) as websocket:

        while True:

            message = await websocket.recv()
            data = json.loads(message)

            if data["type"] != "sensor_summary":
                continue

            # Extract sensor data
            gyro = np.array(data["gyro"])
            accel = np.array(data["accel"])
            depth = data["depth"]

            ret, frame = cap.read()
            if not ret:
                continue

            now = time.time()
            dt = now - prev_time
            prev_time = now

            state = slam.step(frame, gyro, accel, depth, dt)

            print("SLAM Position:", state.position)


if __name__ == "__main__":
    asyncio.run(slam_client())