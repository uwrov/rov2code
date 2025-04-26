import random
import numpy as np
from .rov_config import thruster_config as THRUSTER_CFG
from .accel_gyro_values import manipulate_gyro_accel

from .motor_power_translator import convert_force_and_torque_to_motor_powers
from .pwm_translator import convert_motor_powers_to_pwms

BOTTOM_MANIP_PIN = 11
TOP_MANIP_PIN = 9

TIME_TO_RAMP = 1.0
TIME_PER_CYCLE = 0.1
AMPLITUDE = 400
RAMP_LIMIT = (TIME_PER_CYCLE / TIME_TO_RAMP) * AMPLITUDE


class Core():
    def __init__(self):
        self.interface, self.task = None, None

        self.translate_x = 0.0
        self.translation = [0.0, 0.0, 0.0]
        self.rotation = [0.0, 0.0, 0.0]

        self.direct_motors = False
        self.bottom_manip_pwm = 1500
        self.top_manip_pwm = 1500

        self.accelerometer, self.gyroscope, self.rotational_velocity = None, None, None

        self.prev_pwms = [1500, 1500, 1500, 1500, 1500, 1500]

    def set_interface(self, interface: 'Interface'):
        self.interface = interface

    def set_task(self, task: 'Task'):
        self.task = task

    async def update_sensors(self, packet):
        for key, value in packet.items():
            # print(f"Setting {key}: {value}")
            if hasattr(self, key):
                setattr(self, key, value)
        await self.interface.notify_sensor_update()

    async def update_controls(self):
        trans = self.translation
        rot = self.rotation
        powers = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2]]

        if not self.direct_motors:
            DEADBAND = 0.1
            #print(f"Rot magnitude: {np.linalg.norm(powers[3:])}")
            if np.linalg.norm(powers[3:]) > DEADBAND or self.rotational_velocity is None or self.rotational_velocity == -1:
                print("NO CORRECTION")
                powers = convert_force_and_torque_to_motor_powers(powers)
            else:
                ROT_CORRECTION_STRENGTH = [0.0, 0.0, 0.0]
                # ROT_CORRECTION_STRENGTH = [0.5, 0.5, 0.5]
                print(f"Rot vel: {np.round(self.rotational_velocity, 2)}")
                # max_vel = np.zeros(3)
                # max_vel[np.argmax(np.abs(self.rotational_velocity))] = 1 * np.sign(self.rotational_velocity)[np.argmax(np.abs(self.rotational_velocity))]
                # print(f"Rot vel: {max_vel}")
                powers[3:] = -np.array(self.rotational_velocity) * ROT_CORRECTION_STRENGTH
                powers = convert_force_and_torque_to_motor_powers(powers)
                # print(powers)
        else:
            powers = np.transpose(np.array([powers], dtype=np.float32))

        if False: # PIDF code - doesn't work, just a rudimentary version
            accel_gyro_values.manipulate_gyro_accel_values(self.accelerometer, self.gyroscope)

        for i in range(len(powers)):
            powers[i] *= THRUSTER_CFG[i]['direction']

        # print(powers)
        powers *= 0.25
        largest_power = np.max(np.abs(powers))
        if largest_power > 0.5:
            powers /= largest_power
            powers *= 0.5

        pwms = convert_motor_powers_to_pwms(powers)
        
        delta_pwms = np.subtract(pwms, self.prev_pwms)
        delta_pwms = np.clip(delta_pwms, -RAMP_LIMIT, +RAMP_LIMIT)

        pwms = np.add(self.prev_pwms, delta_pwms).astype(int).tolist()
        self.prev_pwms = pwms

        pin_pwms = [{
            'number': THRUSTER_CFG[i]['pin'],
            'value': pwms[i]
        } for i in range(len(pwms))]

        pin_pwms.append({
            'number': BOTTOM_MANIP_PIN,
            'value': self.bottom_manip_pwm,
        })
        
        pin_pwms.append({
            'number': TOP_MANIP_PIN,
            'value': self.top_manip_pwm,
        })

        return pin_pwms

    async def consume_interface_websocket(self, packet):
        for key, value in packet.items():
            if hasattr(self, key):
                # print(f"Setting {key}: {value}")
                setattr(self, key, value)