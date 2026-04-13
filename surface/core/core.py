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

BOTTOM_MANIP_PIN = 9
TOP_MANIP_PIN = 11

TIME_TO_RAMP = 1.5
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
        self.direct_motors = False
        self.bottom_manip_pwm = 1500
        self.top_manip_pwm = 1500
        
        self.gantry_x = 0.0
        self.gantry_y = 0.0

        self.angular_acceleration, self.accelerometer, self.quaternion, self.rotational_velocity, self.depth, self.gravity_vector = None, None, None, None, None, None
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
        trans[0] *=3 #Strafe should be faster
        rot = self.rotation
        rot[1] *=2 #roll should be faster
        powers = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2]]


        if not self.direct_motors:
            powers = convert_force_and_torque_to_motor_powers(powers)
        else:
            powers = np.transpose(np.array([powers], dtype=np.float32))

        for i in range(len(powers)):
            powers[i] = THRUSTER_CFG[i]['direction']*THRUSTER_CFG[i]['handing']* powers[i] * self.power_scale

        pwms = convert_motor_powers_to_pwms(powers)
        
        delta_pwms = np.subtract(pwms, self.prev_pwms)
        delta_pwms = np.clip(delta_pwms, -RAMP_LIMIT, +RAMP_LIMIT)

        pin_pwms = [{
            'number': THRUSTER_CFG[i]['pin'],
            'value': pwms[i]
        } for i in range(len(pwms))]

        GANTRY_X_PINS = [0, 1]
        GANTRY_Y_PINS = [2, 3]
        GANTRY_PWM_SCALING_FACTOR = 100

        pin_pwms.extend([{
            'number': pin,
            'value': 1500 + self.gantry_x * GANTRY_PWM_SCALING_FACTOR
        } for pin in GANTRY_X_PINS])

        pin_pwms.extend([{
            'number': pin,
            'value': 1500 + self.gantry_y * GANTRY_PWM_SCALING_FACTOR
        } for pin in GANTRY_Y_PINS])

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
