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

def quaternion_to_euler_angle_vectorized1(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z

class ROV:
    def __init__(self):
        self.last_timestamp = time.time()
        self.pi = pigpio.pi()
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)

            try:
                self.secondary_bno = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
            except Exception as e:
                print(e)
                print("UNABLE TO CONNECT TO SECONDARY IMU.")
                self.secondary_bno = None

            self.last_euler = None
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


    # updates accelerometer and gyroscope values
    async def poll_sensors(self):
        now = time.time()
        readings = []
        if self.bno is not None:
            readings.append(self.bno.quaternion)

            # Accelerometer data (in meters per second squared)
            acceleration = np.array(self.bno.linear_acceleration)
            if self.secondary_bno is not None:
                secondary_acceleration = np.array(self.secondary_bno.linear_acceleration)
                if secondary_acceleration[0] is not None and acceleration[0] is not None:
                    #print(secondary_acceleration)
                    secondary_acceleration[1] *= -1
                    secondary_acceleration[2] *= -1
                    #print(f"Primary: {acceleration}")
                    #print(f"Secondary: {secondary_acceleration}")

                    acceleration = np.mean((acceleration, secondary_acceleration), axis=0)
                    #print(f"Combined: {acceleration}")

            readings.append(acceleration.tolist())

            # Gyroscope data (in radians per second)
            rot_vel = self.bno.gyro
            if rot_vel[0] is not None:
                readings.append([rot_vel[0], rot_vel[1], rot_vel[2]])

            if acceleration[0] is not None:
                if self.last_acceleration is not None:
                    #print(f"Acceleration: {acceleration}")

                    calibrate = False
                    if calibrate:
                        self.acceleration_readings.append(acceleration)

                        WINDOW = 1000
                        num_readings = min(WINDOW, len(self.acceleration_readings))
                        print(f"Noise mean: {np.mean(self.acceleration_readings[-num_readings:], axis=0)}")
                        print(f"Noise std: {np.std(self.acceleration_readings[-num_readings:], axis=0)}")


                    if self.kahlman is None:
                        self.linear_velocity += np.array(self.last_acceleration) * (now - self.last_timestamp)
                    else:
                        self.kahlman.predict()
                        self.kahlman.update(acceleration)
                        self.linear_velocity = self.kahlman.x
                    #print(f"Linear velocity: {self.linear_velocity}")

                    #readings.append([self.linear_velocity[0], self.linear_velocity[1], self.linear_velocity[2]])
                else:
                    #readings.append(-1)
                    pass

                self.last_acceleration = acceleration
            else:
                #readings.append(-1)
                pass

        else:
            readings.append(-1)
            readings.append(-1)
            readings.append(-1)

        self.last_timestamp = now
        return readings
