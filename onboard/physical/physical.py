import busio
import board
import time
from gpiozero import Servo
import ms5837

class ROV:
    def __init__(self):
        self.servos = {}
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bar02 = ms5837.MS5837_02BA()
        except Exception as e:
            print(e)
            print("UNABLE TO CONNECT TO IMU")

    # number = GPIO number
    # value = PWM value
    def set_pin_pwm(self, number: int, value: int):
        # print(number, value)
        if number not in self.servos:
            self.servos[number] = Servo(number)

        normalized = (value - 1500) / 500
        self.servos[number].value = max(min(normalized, 1), -1)

    # unnecessary for physical ROV
    async def flush_pin_pwms(self):
        pass

    # get depth function, heavily based on quinns imu function
    def get_depth(self) -> dict:
        if self.bar02 is None:
            return {}
        try:
            depth = self.bar02.depth
            if depth[0] is None:
                return {}
            return {"depth": depth}
        except:
            return {}

    async def poll_sensors(self) -> dict:
        readings = []

        readings.append(self.get_depth)

        # Will be a list of (maybe empty) dictionaries of readings to report
        readings_dict = {}
        for reading in readings:
            readings_dict.update(reading)

        return readings_dict
