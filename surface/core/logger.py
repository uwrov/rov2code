import csv
import os
from datetime import datetime


class ROVLogger:
    def __init__(self, log_dir="logs"):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs(log_dir, exist_ok=True)
        self.file_path = os.path.join(log_dir, f"log_{timestamp}.csv")
        self.file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self._write_header()
        


    def _write_header(self):
        self.writer.writerow([
            "time",
            "ay", "ax", "az",
            "gy", "gx", "gz",
            "roll", "pitch", "yaw",
            "cmd_fx", "cmd_fy", "cmd_fz",
            "cmd_tx", "cmd_ty", "cmd_tz",
            "pwm_1", "pwm_2", "pwm_3", "pwm_4", "pwm_5", "pwm_6"
        ])

    def log(self, timestamp, imu_data, orientation, cmd_force, cmd_torque, pwm_outputs):
        self.writer.writerow([
            timestamp,
            *imu_data['accel'],     # ay, ax, az
            *imu_data['gyro'],      # gy, gx, gz
            *orientation,           # roll, pitch, yaw (or euler angles)
            *cmd_force,             # fx, fy, fz
            *cmd_torque,            # 
            *pwm_outputs[:6]        # per-thruster pwm values
        ])

    def close(self):
        self.file.close()
