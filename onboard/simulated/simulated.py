import random
import sys
import os

sys.path.append(os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../surface/core")
))

from rov_config import thruster_config
from rov_config import motor_config

PIN_HACK = 1500

class ROV:
    def __init__(self):
        print('initializing simulated ROV')
        self.pwms = {}
        # TODO start Godot simulation and ensure cameras are connected


    def set_pin_pwm(self, number: int, value: int):
        self.pwms[number] = value 
        fraction = (value - 1500) / 400.0
        # print(f'setting pwm: pin {number} at {value} µs ({(value - 1500):+4d}, {fraction:+7.3f})')
        if number == 10:
            PIN_HACK = value
        # TODO: implement


    async def flush_pin_pwms(self):
        print()  # add separation in terminal between groups of PWM setting
        pass
        # TODO: implement sending to simulation


    async def poll_sensors(self):
        accelerometer = [(1500-PIN_HACK) * 0.01, 0.0, round(-9.81 + random.randint(-5, 5) * 0.01, 2)]
        gyroscope = [0.0, 0.0, 0.0]
        thrusters = {}
        motors = {}
        for t in thruster_config:
            pin = t["pin"]
            name = t["name"]
            pwm = self.pwms.get(pin, 1500)

            thrust = (pwm - 1500)

            thrust *= t.get("direction", 1)
            thrust *= t.get("handing", 1)

            thrusters[name] = thrust
        for m in motor_config:
            pin = m["pin"]
            name = m["name"]
            pwm = self.pwms.get(pin, 1500)

            motor_value = (pwm - 1500)

            motor_value *= m.get("direction", 1)

            motors[name] = motor_value
        # TODO implement retrieving from simulation
        # kinda being done
        return {
            "quaternion": gyroscope,       # match Godot expectation
            "accelerometer": accelerometer,
            "thrusters": thrusters,
            "motors": motors,
        }