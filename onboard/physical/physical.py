import pigpio
import logging
import sys
import time
import serial
import busio
import board
import numpy as np
import time

import adafruit_bno055

class ROV:
    def __init__(self):
        self.last_timestamp = time.time()
        self.pi = pigpio.pi()
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)

            try:
                self.secondary_bno = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
                self.secondary_bno = None
            except Exception as e:
                print(e)
                print("UNABLE TO CONNECT TO SECONDARY IMU.")
                self.secondary_bno = None

            self.last_acceleration = np.zeros(3)
            self.linear_velocity = np.zeros(3)

            self.acceleration_readings = []

            #Noise mean: [ 0.00105 -0.03796 -0.02601]
            #Noise std: [0.03169223 0.0771689  0.06144087]
            accel_std = None

            if accel_std is not None:
                self.kahlman = KalmanFilter(dim_x=3, dim_z=3)

                # Initial state
                self.kahlman.x = np.zeros(3)

                # Transition matrix
                self.kahlman.F = np.eye(3)

                # Measurement function
                self.kahlman.H = np.ones(3)

                # Covariance matrix
                self.kahlman.P *= 1000

                # Measurement noise
                self.kahlman.R = np.eye(3) * (accel_std ** 2)

                # Process noise
                self.kahlman.Q = np.eye(3) * 0.01
            else:
                self.kahlman = None

            # UART setup
            # uart = serial.Serial("/dev/serial0")
            # self.bno = adafruit_bno055.BNO055_UART(uart)
        # self.bno = None
        except Exception as e:
            print(e)
            print("UNABLE TO CONNECT TO IMU")
            self.bno = None

    # number = GPIO number
    # value = PWM value
    def set_pin_pwm(self, number: int, value: int): 
        # print(number, value)
        self.pi.set_servo_pulsewidth(number, value)


    # unnecessary for physical ROV
    async def flush_pin_pwms(self):
        pass

    # Not good!
    def get_linear_velocity(self) -> dict:
        if self.bno is None:
            return {}

        # Accelerometer data (in meters per second squared)
        acceleration = np.array(self.bno.linear_acceleration)

        if self.secondary_bno is not None:
            secondary_acceleration = np.array(self.secondary_bno.linear_acceleration)

            if secondary_acceleration[0] is not None:
                secondary_acceleration[1] *= -1
                secondary_acceleration[2] *= -1

        if acceleration[0] is None:
            if self.secondary_bno is not None and secondary_acceleration[0] is not None:
                acceleration = secondary_acceleration
            else:
                # No good acceleration data, use our current linear_velocity
                return {"linear_velocity": self.linear_velocity.tolist()}

        if self.secondary_bno is not None and secondary_acceleration[0] is not None:
            # Good data from both!
            acceleration = np.mean(
                (acceleration,
                secondary_acceleration), axis=0)

        if self.last_acceleration is not None:
            # Lay flat and get ambient noise
            calibrate = False
            if calibrate:
                self.acceleration_readings.append(acceleration)

                WINDOW = 1000
                num_readings = min(WINDOW, len(self.acceleration_readings))
                print(f"Noise mean: {np.mean(self.acceleration_readings[-num_readings:], axis=0)}")
                print(f"Noise std: {np.std(self.acceleration_readings[-num_readings:], axis=0)}")

            now = time.time()
            if self.kahlman is None:
                # Do dumb interpolation if no Kahlman
                self.linear_velocity += np.array(self.last_acceleration) * (now - self.last_timestamp)
            else:
                self.kahlman.predict()
                self.kahlman.update(acceleration)
                self.linear_velocity = self.kahlman.x

            self.last_acceleration = acceleration
            self.last_timestamp = now

        return {"linear_velocity": self.linear_velocity.tolist()}

    def get_quaternion(self) -> dict:
        if self.bno is None:
            return {}
        
        quat = self.bno.quaternion
        if quat[0] is None:
            return {}
        
        return {"quaternion": quat}

    def get_angular_velocity(self) -> dict:
        if self.bno is None:
            return {}

        rot_vel = self.bno.gyro

        if self.secondary_bno is not None:
            secondary_rot_vel = self.secondary_bno.gyro

        # Will be None if we get a bad reading
        if rot_vel[0] is None or np.max(np.abs(rot_vel)) > 5:
            # Primary is bad, fall back on secondary
            if self.secondary_bno is not None and secondary_rot_vel[0] is not None:
                return {"rotational_velocity": [secondary_rot_vel[0], secondary_rot_vel[1], secondary_rot_vel[2]]}
            else:
                return {}
        print(np.round(rot_vel, 2))

        if self.secondary_bno is not None and secondary_rot_vel[0] is not None:
            # Good data from both!
            return {"rotational_velocity": np.mean(
                ([rot_vel[0], rot_vel[1], rot_vel[2]],
                [secondary_rot_vel[0], secondary_rot_vel[1], secondary_rot_vel[2]]), axis=0).tolist()
            }
        else:
            # Bad data from secondary, or we don't have one.
            return {"rotational_velocity": [rot_vel[0], rot_vel[1], rot_vel[2]]}

    async def poll_sensors(self) -> dict:
        readings = []

        readings.append(self.get_quaternion())
        readings.append(self.get_angular_velocity())
        readings.append(self.get_linear_velocity())

        # Will be a list of (maybe empty) dictionaries of readings to report
        readings_dict = {}
        for reading in readings:
            readings_dict.update(reading)

        return readings_dict
