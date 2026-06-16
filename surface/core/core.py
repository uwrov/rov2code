import random
import numpy as np
import pickle
from .rov_config import thruster_config as THRUSTER_CFG
from .rov_config import motor_config as MOTOR_CFG
from .accel_gyro_values import manipulate_gyro_accel

from .motor_power_translator import convert_force_and_torque_to_motor_powers
from .force_to_pwm import convert_motor_powers_to_pwms
from .logger import ROVLogger
from .rov_config import imu_position as IMU_POS

import time
import math

import cv2

logger = ROVLogger()
TIME_TO_RAMP = 1.0
TIME_PER_CYCLE = 0.1 # dont change
AMPLITUDE = 400
RAMP_LIMIT = (TIME_PER_CYCLE / TIME_TO_RAMP) * AMPLITUDE

cam = cv2.VideoCapture("http://172.25.250.1:8554/")
img_counter = 0

class Core():
    def __init__(self):
        self.interface, self.task = None, None

        self.translate_x = 0.0
        self.translation = [0.0, 0.0, 0.0]
        self.rotation = [0.0, 0.0, 0.0]
        self.override = [1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]
        self.direct_motors = False
        self.power_scale = 0.5
        self.manipulator_pwm = 1500
        self.right_gantry = 1500
        self.left_gantry = 1500
        self.buoyancy_arm = 1500
        self.capture_frame = False

        self.angular_acceleration, self.accelerometer, self.thrusters, self.gantry,self.motors, self.quaternion, self.rotational_velocity, self.depth, self.gravity_vector, self.arm_angle =None, None,None,None, None, None, None, None, None, None
        self.rotational_velocity_accum = np.zeros(3)
        self.last_rotational_velocity = None
        self.last_depth = None

        self.depth_i = 0.0
        self.depth_prev_error = 0.0
        self.depth_prev_time = None

        self.cam = cv2.VideoCapture("http://172.25.250.1:8554/")

        self.prev_pwms = [1500, 1500, 1500, 1500, 1500, 1500]

    def set_interface(self, interface: 'Interface'):
        self.interface = interface

    def set_task(self, task: 'Task'):
        self.task = task

    async def update_sensors(self, packet):
        for key, value in packet.items():
            if hasattr(self, key):
                setattr(self, key, value)
        await self.interface.notify_sensor_update()

    async def update_controls(self):
        trans = self.translation
        # trans[0] *=3 #Strafe should be faster
        rot = self.rotation
        # rot[1] *=2 #roll should be faster     
        powers = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2]]  

        if self.capture_frame:
            ret, frame = cam.read()   
            cv2.imwrite("../analysis/", frame)
            img_counter += 1
            print("Captured")
            
        if not self.direct_motors:
            DEADBAND = 0.1
            depth_hold = True
            if abs(trans[2]) > DEADBAND or self.depth is None:
                # If we're manually controlling depth or missing sensor data, disable depth hold
                powers = convert_force_and_torque_to_motor_powers(powers)

                if self.depth is not None:
                    self.last_depth = np.array(self.depth)

                self.depth_i = 0.0
                self.depth_prev_error = 0.0
                self.depth_prev_time = None

            elif depth_hold and self.last_depth is None:
                self.last_depth = np.array(self.depth)
                self.depth_i = 0.0
                self.depth_prev_error = 0.0
                self.depth_prev_time = None
                powers = convert_force_and_torque_to_motor_powers(powers)
            
            elif depth_hold and self.last_depth is not None:
                # If not controlling depth, we hold depth using PID
                now = time.time()

                if self.depth_prev_time is None:
                    self.depth_prev_time = now

                dt = now - self.depth_prev_time
                self.depth_prev_time = now

                error = float(self.last_depth - self.depth)

                self.depth_i += error * dt
                self.depth_i = np.clip(self.depth_i, -2.0, 2.0)

                if dt > 0:
                    d_error = (error - self.depth_prev_error) / dt
                else:
                    d_error = 0.0

                self.depth_prev_error = error

                DEPTH_P = 0.5
                DEPTH_I = 0.0
                DEPTH_D = 0.1

                correction = (
                    DEPTH_P * error
                    + DEPTH_I * self.depth_i
                    + DEPTH_D * d_error
                )

                correction = np.clip(correction, -0.5, 0.5)

                powers[2] += correction

                powers = convert_force_and_torque_to_motor_powers(powers)

        else:
            powers = np.transpose(np.array([powers], dtype=np.float32))

        for i in range(len(powers)):
            powers[i] = powers[i] * self.power_scale
        pwms = convert_motor_powers_to_pwms(powers)
        pwms = np.array(pwms, dtype=float)
        prev = np.array(self.prev_pwms, dtype=float)
        delta_pwms = np.subtract(pwms,prev)
        delta_pwms = np.clip(delta_pwms, -RAMP_LIMIT, +RAMP_LIMIT)
        pwms = np.add(prev, delta_pwms)
        self.prev_pwms = pwms.tolist()

        pin_pwms = [{
            'number': THRUSTER_CFG[i]['pin'],
            'value': pwms[i]
        } for i in range(len(pwms))]

        pin_pwms += [
            {
                'number': MOTOR_CFG[0]['pin'], #arm
                'value': 1500 + int(rot[0] * 100)
            },
            {
                'number': MOTOR_CFG[1]['pin'],
                'value': int(self.right_gantry)
            },
            {
                'number': MOTOR_CFG[2]['pin'],
                'value': int(self.left_gantry)
            },
            {
                'number': MOTOR_CFG[3]['pin'],
                'value': self.manipulator_pwm
            }
        ]
        
        if self.direct_motors : 
            pin_pwms = [
                {
                    'number': THRUSTER_CFG[0]['pin'],
                    'value': self.override["motor_a"]
                },
                {
                    'number': THRUSTER_CFG[1]['pin'],
                    'value': self.override["motor_b"]
                },
                {
                    'number': THRUSTER_CFG[2]['pin'],
                    'value': self.override["motor_c"]
                },
                {
                    'number': THRUSTER_CFG[3]['pin'],
                    'value': self.override["motor_d"]
                },
                {
                    'number': THRUSTER_CFG[4]['pin'],
                    'value': self.override["motor_e"]
                },
                {
                    'number': THRUSTER_CFG[5]['pin'],
                    'value': self.override["motor_f"]
                },
                {
                    'number': MOTOR_CFG[0]['pin'],
                    'value': self.override["motor_g"]
                },
                {
                    'number': MOTOR_CFG[1]['pin'],
                    'value': self.override["motor_h"]
                },
                {
                    'number': MOTOR_CFG[2]['pin'],
                    'value': self.override["motor_i"]
                },
                {
                    'number': MOTOR_CFG[3]['pin'],
                    'value': self.override["motor_j"]
                }
            ]   
        
        pwm_sum=0
        for p in pin_pwms:
            pwm_sum += abs(1500 - p['value'])
        if pwm_sum > 1200 :
            for i in range(len(pin_pwms)):
                pin_pwms[i]['value'] = 1500 + (pin_pwms[i]['value'] - 1500) * 1200 / pwm_sum
        return pin_pwms

    async def consume_interface_websocket(self, packet):
        for key, value in packet.items():
            if hasattr(self, key):
                setattr(self, key, value)

def quaternion_to_euler(q):
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90° if out of range
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)
