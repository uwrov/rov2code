import random

from rov_base import ROVBase

PIN_HACK = 1500
class ROV(ROVBase):
    def __init__(self):
        super().__init__()
        print('initializing simulated ROV')
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

        # TODO implement retrieving from simulation
        # kinda being done
        return {
            "quaternion": gyroscope,       # match Godot expectation
            "accelerometer": accelerometer,
            **self.actuator_readings(),
        }