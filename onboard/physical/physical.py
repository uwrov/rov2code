import busio
import board
import time
from gpiozero import Servo
import adafruit_bno055
import np

class ROV:
    def __init__(self):
        self.servos = {}
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
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

    def get_quaternion(self) -> dict:
        if self.bno is None:
            return {}
        
        try:
            quat = self.bno.quaternion
            if quat[0] is None:
                return {}

            return {"quaternion": quat}
        except:
            return {}

    def get_linear_acceleration(self) -> dict:
        if self.bno is None:
            return {}

        try:
            accel = np.array(self.bno.linear_acceleration)
        
            if accel[0] is None:
                return {}

            return {"accelerometer": accel}
        except:
            return {}

    async def poll_sensors(self) -> dict:
        readings = []

        readings.append(self.get_quaternion())
        readings.append(self.get_linear_acceleration())

        # Will be a list of (maybe empty) dictionaries of readings to report
        readings_dict = {}
        for reading in readings:
            readings_dict.update(reading)

        return readings_dict
