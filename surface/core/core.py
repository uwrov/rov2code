import random
import numpy as np
from .rov_config import thruster_config as THRUSTER_CFG
from .rov_config import motor_config as MOTOR_CFG

from .motor_power_translator import convert_force_and_torque_to_motor_powers
from .force_to_pwm import convert_motor_powers_to_pwms

import time

import os

import cv2

NEUTRAL_PWM = 1500
TIME_TO_RAMP = 1.0
TIME_PER_CYCLE = 0.1 # dont change
AMPLITUDE = 400
RAMP_LIMIT = (TIME_PER_CYCLE / TIME_TO_RAMP) * AMPLITUDE

CONTROL_TIMEOUT = 1.0  # seconds without Godot input before controls go neutral

CAMERA_PORTS = (8554, 8555, 8556)

# Godot's direct-motor keys, in order: thrusters first, then auxiliary motors
OVERRIDE_PINS = {
    f"motor_{letter}": cfg['pin']
    for letter, cfg in zip("abcdefghij", (*THRUSTER_CFG, *MOTOR_CFG))
}

SENSOR_FIELDS = (
    "angular_acceleration", "accelerometer", "thrusters", "gantry", "motors",
    "quaternion", "rotational_velocity", "depth", "gravity_vector", "arm_angle",
)

class Core():
    def __init__(self):
        self.interface = None

        self.translate_x = 0.0
        self.power_scale = 0.5
        self.set_controls_neutral()

        # update_sensors only sets attributes that already exist
        for name in SENSOR_FIELDS:
            setattr(self, name, None)
        self.rotational_velocity_accum = np.zeros(3)
        self.last_rotational_velocity = None
        self.last_depth = None

        self.depth_i = 0.0
        self.depth_prev_error = 0.0
        self.depth_prev_time = None
        
        self.cams = {port: cv2.VideoCapture(f"http://172.25.250.1:{port}/") for port in CAMERA_PORTS}
        self.img_counter = 0

        self.prev_pwms = [NEUTRAL_PWM] * len(THRUSTER_CFG)

    def set_interface(self, interface: 'Interface'):
        self.interface = interface

    def set_controls_neutral(self):
        self.translation = [0.0] * 3
        self.rotation = [0.0] * 3
        self.override = dict.fromkeys(OVERRIDE_PINS, NEUTRAL_PWM)
        self.direct_motors = False
        self.depth_hold = False
        self.manipulator_pwm = self.right_gantry = self.left_gantry = self.buoyancy_arm = NEUTRAL_PWM
        self.capture_frame = False
        self.last_control_time = None

    async def update_sensors(self, packet):
        for key, value in packet.items():
            if hasattr(self, key):
                setattr(self, key, value)
        await self.interface.notify_sensor_update()

    async def update_controls(self):
        if self.last_control_time is not None and time.monotonic() - self.last_control_time > CONTROL_TIMEOUT:
            print('Godot input timed out, setting controls to neutral')
            self.set_controls_neutral()

        trans = self.translation
        # trans[0] *=3 #Strafe should be faster
        rot = self.rotation
        # rot[1] *=2 #roll should be faster     
        powers = [*trans, *rot]  
        # print(powers)
        # print(self.capture_frame)
        # if self.capture_frame:            
        #         print("Space")
        try:
            frames = {}
            for port, cam in self.cams.items():
                ret, frame = cam.read()
                if ret:
                    frames[port] = frame
            if self.capture_frame:            
                print("Space")
                for port, frame in frames.items():
                    filename = f"D:/cam_{port}_frame_" + str(self.img_counter) + ".png"
                    print(os.getcwd())
                    success = cv2.imwrite(filename, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
                    self.img_counter += 1
                    if success:
                        print("Yes")
                    else:
                        print("Couldnt save to thumb drive, saving to surface/analysis")
                        filename = r"C:\Users\uwrov\Documents\GitHub\rov2code\surface\analysis\cam_{port}_frame_" + str(self.img_counter) + ".png"
                        cv2.imwrite(filename, frame)
                else:
                    print("Failed to capture frame")
                
                self.capture_frame = False
        finally:
            pass
            
        # Depth Hold Block
        if not self.direct_motors:
            DEADBAND = 0.1

            if self.depth_hold and self.depth is not None and abs(trans[2]) <= DEADBAND:
                if self.last_depth is None:
                    # First cycle of depth hold to initialize
                    self.last_depth = np.array(self.depth)
                    self.depth_i = 0.0
                    self.depth_prev_error = 0.0
                    self.depth_prev_time = None

                else:
                    # If not controlling depth, we hold depth using PID
                    
                    # These are currently arbitrary values (NEED TUNING!)
                    # scaled by 100 since error is in meters
                    DEPTH_P = 2000
                    DEPTH_I = 0.0
                    DEPTH_D = 0
                    
                    now = time.time()

                    if self.depth_prev_time is None:
                        self.depth_prev_time = now

                    dt = now - self.depth_prev_time
                    self.depth_prev_time = now

                    error = np.array(self.depth) - self.last_depth
                    print ("Depth Error: ", error)

                    if np.abs(error) > 0.01:
                        self.depth_i += error * dt
                        self.depth_i = np.clip(self.depth_i, -2.0, 2.0)

                        if dt > 0:
                            d_error = (error - self.depth_prev_error) / dt
                        else:
                            d_error = 0.0

                        self.depth_prev_error = error

                        correction = (
                            DEPTH_P * error
                            + DEPTH_I * self.depth_i
                            + DEPTH_D * d_error
                        )

                        correction = np.clip(correction, -1.0, 1.0)*-1500

                        powers[2] = correction

                    else:
                        powers[2] = 0.0
                        self.depth_prev_error = error
                
            else:
                # If we're manually controlling depth or missing sensor data, disable depth hold
                if self.depth is not None:
                    self.last_depth = np.array(self.depth)

                self.depth_i = 0.0
                self.depth_prev_error = 0.0
                self.depth_prev_time = None
            
            powers = convert_force_and_torque_to_motor_powers(powers)

        else:
            powers = np.array(powers, dtype=np.float32)

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
                'value': NEUTRAL_PWM + int(rot[0] * 100)
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
                {'number': pin, 'value': self.override[key]}
                for key, pin in OVERRIDE_PINS.items()
            ]   
        
        pwm_sum=0
        for p in pin_pwms:
            pwm_sum += abs(NEUTRAL_PWM - p['value'])
        if pwm_sum > 3400 :
            for i in range(len(pin_pwms)):
                pin_pwms[i]['value'] = NEUTRAL_PWM + (pin_pwms[i]['value'] - NEUTRAL_PWM) * 1200 / pwm_sum
        return pin_pwms

    async def consume_interface_websocket(self, packet):
        for key, value in packet.items():
            if hasattr(self, key):
                setattr(self, key, value)
        self.last_control_time = time.monotonic()
