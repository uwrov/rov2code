import asyncio

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo, Device
Device.pin_factory = PiGPIOFactory()
from physical.drivers.ms5837 import ms5837
from rov_base import ROVBase

class ROV(ROVBase):
    def __init__(self):
        super().__init__()
        self.servos = {}
        self.bar02 = self.connect_depth_sensor()

    def connect_depth_sensor(self):
        try:
            sensor = ms5837.MS5837_02BA()
            if sensor.init():
                return sensor
        except Exception as e:
            print(e)

        print("UNABLE TO CONNECT TO DEPTH SENSOR")
        return None

    # number = GPIO number
    # value = PWM value
    def set_pin_pwm(self, number: int, value: int):
        if number not in self.servos:
            self.servos[number] = Servo(number)

        normalized = (value - 1500) / 500
        self.servos[number].value = max(min(normalized, 1), -1)
        self.pwms[number] = value

    # unnecessary for physical ROV
    async def flush_pin_pwms(self):
        pass

    def get_depth(self) -> dict:
        if self.bar02 is None:
            return {}

        try:
            if self.bar02.read():
                return {"depth": self.bar02.depth()}
        except Exception as e:
            print(f'depth read failed: {e!r}')

        return {}

    async def poll_sensors(self) -> dict:
        return {
            # the read sleeps for about 41 ms, so keep it off the event loop handling commands
            **await asyncio.to_thread(self.get_depth),
            # TODO placeholders since the IMU was removed
            "quaternion": [1.0, 0.0, 0.0],
            "accelerometer": [0.0, 0.0, 0.0],
            **self.actuator_readings(),
        }
