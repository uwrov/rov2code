import threading
import time

class SLAMBridge:

    def __init__(self, slam_system, sensor_provider):

        self.slam = slam_system
        self.sensor_provider = sensor_provider
        self.running = False

    def start(self):

        self.running = True
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):

        prev_time = time.time()

        while self.running:

            frame, gyro, accel, depth = self.sensor_provider.get_data()

            now = time.time()
            dt = now - prev_time
            prev_time = now

            self.slam.step(frame, gyro, accel, depth, dt)