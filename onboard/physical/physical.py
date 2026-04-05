import pigpio
import time
import busio
import board
import time

class ROV:
    def __init__(self):
        self.last_timestamp = time.time()
        self.pi = pigpio.pi()
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
        except Exception as e:
            print(e)
            print("UNABLE TO CONNECT TO IMU")

    # number = GPIO number
    # value = PWM value
    def set_pin_pwm(self, number: int, value: int): 
        # print(number, value)
        self.pi.set_servo_pulsewidth(number, value)

    def set_pin(self, number: int, value: bool):
        self.pi.write(number, pigpio.HIGH if value else pigpio.LOW)
        print(number)
        print(value)

    # unnecessary for physical ROV
    async def flush_pin_pwms(self):
        pass

    async def poll_sensors(self) -> dict:
        readings = []

        # Will be a list of (maybe empty) dictionaries of readings to report
        readings_dict = {}
        for reading in readings:
            readings_dict.update(reading)

        return readings_dict
