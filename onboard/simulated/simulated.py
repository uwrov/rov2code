import random

from rov_base import ROVBase

class ROV(ROVBase):
    def __init__(self):
        super().__init__()
        print('initializing simulated ROV')
        # TODO start Godot simulation and ensure cameras are connected


    def set_pin_pwm(self, number: int, value: int):
        self.pwms[number] = value
        # TODO: implement sending to simulation


    async def poll_sensors(self):
        accelerometer = [0.0, 0.0, round(-9.81 + random.randint(-5, 5) * 0.01, 2)]
        quaternion = [1.0, 0.0, 0.0, 0.0]  # level, in the IMU's w, x, y, z order

        # TODO implement retrieving from simulation
        # kinda being done
        return {
            "quaternion": quaternion,      # match Godot expectation
            "accelerometer": accelerometer,
            **self.actuator_readings(),
        }