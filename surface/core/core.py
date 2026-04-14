import random
import numpy as np
import pickle
from .rov_config import thruster_config as THRUSTER_CFG
from .accel_gyro_values import manipulate_gyro_accel

from .motor_power_translator import convert_force_and_torque_to_motor_powers
# from .pwm_translator import convert_motor_powers_to_pwms #old linear one
from .force_to_pwm import convert_motor_powers_to_pwms
from .logger import ROVLogger
from .rov_config import imu_position as IMU_POS

import time
import math

logger = ROVLogger()
TIME_TO_RAMP = 0.5
TIME_PER_CYCLE = 0.1
AMPLITUDE = 400
RAMP_LIMIT = (TIME_PER_CYCLE / TIME_TO_RAMP) * AMPLITUDE


class Core():
    def __init__(self):
        self.interface, self.task = None, None

        self.translate_x = 0.0
        self.translation = [0.0, 0.0, 0.0]
        self.rotation = [0.0, 0.0, 0.0]
        self.power_scale = 0.5
        self.manipulator_pwm = 1500
        self.right_gantry = 1500
        self.left_gantry = 1500
        self.buoyancy_arm = 1500

        self.angular_acceleration, self.accelerometer, self.thrusters, self.gantry,self.motors, self.quaternion, self.rotational_velocity, self.depth, self.gravity_vector, self.arm_angle =None, None,None,None, None, None, None, None, None, None
        self.rotational_velocity_accum = np.zeros(3)
        self.last_rotational_velocity = None
        self.last_depth = None

        self.prev_pwms = [1500, 1500, 1500, 1500, 1500, 1500]

    def set_interface(self, interface: 'Interface'):
        self.interface = interface

    def set_task(self, task: 'Task'):
        self.task = task

    async def update_sensors(self, packet):
        # print("Sensor update keys:", packet.keys())
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

        powers = convert_force_and_torque_to_motor_powers(powers)
        
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

        # GANTRY_PINS = [0, 1]
        # GANTRY_PWM_SCALING_FACTOR = 100

        # top_right_pwm = int((self.gantry_x + self.gantry_y) * GANTRY_PWM_SCALING_FACTOR)
        # top_left_pwm = int((-self.gantry_x + self.gantry_y) * GANTRY_PWM_SCALING_FACTOR) 

        # pin_pwms.append({'number': GANTRY_PINS[0], 'value': top_right_pwm})
        # pin_pwms.append({'number': GANTRY_PINS[1], 'value': top_left_pwm})

        # pin_pwms = [26, 19, 13, 6, 11, 9, 20, 16, 12, 25]
        # gantry: 26 (right_gantry), 9 (left_gantry)
        # manipulator: 12
        # buoyancy arm: 25
        # maybe problematic: 11 (bottom), 20 (left_up (not escs))
        BUOYANCY_ARM_PIN = 25
        GANTRY_RIGHT_PIN = 26
        GANTRY_LEFT_PIN = 9
        MANIPULATOR_PIN = 12
        pin_pwms += [
            {
                'number': BUOYANCY_ARM_PIN, 
                'value': 1500 + int(rot[0] * 200)
            },
            {
                'number': GANTRY_RIGHT_PIN,
                'value': int(self.right_gantry)
            },
            {
                'number': GANTRY_LEFT_PIN,
                'value': int(self.left_gantry)
            },
            {
                'number': MANIPULATOR_PIN,
                'value': self.manipulator_pwm
            }
        ]

        return pin_pwms

    async def consume_interface_websocket(self, packet):
        for key, value in packet.items():
            if hasattr(self, key):
                # print(f"Setting {key}: {value}")
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
