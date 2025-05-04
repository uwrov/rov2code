import random
import numpy as np
import pickle
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

        self.accelerometer, self.quaternion, self.rotational_velocity, self.depth, self.gravity_vector = None, None, None, None, None
        self.rotational_velocity_accum = np.zeros(3)
        self.last_rotational_velocity = None
        self.last_depth = None

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
            depth_hold = True
            if np.linalg.norm(powers) > DEADBAND or self.rotational_velocity is None or self.last_depth is None:
                powers = convert_force_and_torque_to_motor_powers(powers)
                self.rotational_velocity_accum = np.zeros(3)
                if self.depth is not None:
                    self.last_depth = np.array(self.depth)
                    # print(self.depth)
            elif depth_hold and self.last_depth is not None:
                DEPTH_CORRECTION_P = [0.0, 0.0, 0.0]
                # error = np.array(self.depth) - self.last_depth
                # print(error)

                # if np.abs(error) > 0.5:
                #     correction_vector = error * np.array(self.gravity_vector)
                #     powers[:3] = correction_vector * DEPTH_CORRECTION_P
                #     # powers[3:] = np.zeros(3)
                # print(np.round(powers, 2))
                # powers = convert_force_and_torque_to_motor_powers(powers)

            # else:
                # ROT_CORRECTION_P = [0.25, 0.25, 0.5]
                # ROT_CORRECTION_I = [0.05, 0.025, 0.05]
                # ROT_CORRECTION_D = [0.2, 0.5, 1.0]
                ROT_CORRECTION_P = [0.,0.,0.]
                ROT_CORRECTION_I = [0.,0.,0.]
                ROT_CORRECTION_D = [0.,0.,0.]
                # print(self.rotational_velocity_accum)
                self.rotational_velocity_accum += np.array(self.rotational_velocity)
                p_term = np.array(self.rotational_velocity) * ROT_CORRECTION_P
                i_term = self.rotational_velocity_accum * ROT_CORRECTION_I

                if self.last_rotational_velocity is not None:
                    d_term = (np.array(self.rotational_velocity) - self.last_rotational_velocity) * ROT_CORRECTION_D
                else:
                    d_term = 0.0

                self.last_rotational_velocity = np.array(self.rotational_velocity)
                # print( -(p_term + i_term + d_term))
                powers[3:] = -(p_term + i_term + d_term)
                # print(np.round(self.rotational_velocity, 2))
                # print(f"powers: {np.round(powers[3:], 2)}")

                pickle.dump(self.rotational_velocity, open("core/rot.pkl", 'wb'))
                pickle.dump(0.0, open("core/setpoint.pkl", 'wb'))
                powers = convert_force_and_torque_to_motor_powers(powers)
        else:
            powers = np.transpose(np.array([powers], dtype=np.float32))

        if False: # PIDF code - doesn't work, just a rudimentary version
            accel_gyro_values.manipulate_gyro_accel_values(self.accelerometer, self.gyroscope)

        for i in range(len(powers)):
            powers[i] *= THRUSTER_CFG[i]['direction']

        powers *= 0.25
        largest_power = np.max(np.abs(powers))
        if largest_power > 0.5:
            powers /= largest_power
            powers *= 0.5

        powers *= 0.75
        print(np.round(np.transpose(powers), 2))
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
