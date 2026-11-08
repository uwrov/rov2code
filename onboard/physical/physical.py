import asyncio

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo, Device
Device.pin_factory = PiGPIOFactory()
from physical.drivers.ms5837 import ms5837
from rov_base import ROVBase

# telemetry key -> BNO055 property
IMU_READINGS = {
    "quaternion": "quaternion",
    "accelerometer": "linear_acceleration",
    "gravity_vector": "gravity",
    "rotational_velocity": "gyro",
}

class ROV(ROVBase):
    def __init__(self):
        super().__init__()
        self.servos = {}
        self.bno = self.connect_imu()
        self.bar02 = self.connect_depth_sensor()

    def connect_imu(self):
        try:
            import adafruit_bno055
            import board
            import busio
            return adafruit_bno055.BNO055_I2C(busio.I2C(board.SCL, board.SDA), address=0x29)
        except Exception as e:
            print(e)

        print("UNABLE TO CONNECT TO IMU")
        return None

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

    def get_depth(self) -> dict:
        if self.bar02 is None:
            return {}

        try:
            if self.bar02.read():
                return {"depth": self.bar02.depth()}
        except Exception as e:
            print(f'depth read failed: {e!r}')

        return {}

    def get_imu(self) -> dict:
        if self.bno is None:
            return {}

        readings = {}
        for key, imu_property in IMU_READINGS.items():
            try:
                value = getattr(self.bno, imu_property)
                if None not in value:
                    readings[key] = list(value)
            except Exception as e:
                print(f'IMU read failed ({imu_property}): {e!r}')

        if "gravity_vector" in readings:
            x, y, z = readings["gravity_vector"]
            readings["gravity_vector"] = [-x, -y, z]

        return readings

    def read_sensors(self) -> dict:
        return {**self.get_depth(), **self.get_imu()}

    async def poll_sensors(self) -> dict:
        return {
            # the depth read sleeps for about 41 ms, so keep it off the event loop handling commands
            **await asyncio.to_thread(self.read_sensors),
            **self.actuator_readings(),
        }
