import busio
import board
import time
import numpy as np
from .rov_config import thruster_config
from .rov_config import motor_config

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo, Device
Device.pin_factory = PiGPIOFactory()
from onboard.physical.drivers.ms5837 import ms5837
import adafruit_bno055

class ROV:
    def __init__(self):
        self.servos = {}
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
        except Exception as e:
            print(e)
            print("UNABLE TO CONNECT TO IMU")


        try:
            self.bar02 = ms5837.MS5837_02BA()
            if not self.bar02.init():
                raise Exception()
        except Exception as e:
            print(e)
            print("UNABLE TO CONNECT TO DEPTH SENSOR")
            self.bar02 = None

        
        self.pwms = {}
        self.gantry = {"x": 0.0, "y": 0.0}
        self.arm_angle = 0.0

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

    def get_depth(self) -> dict:
        if self.bar02 is None:
            return {}

        try:
            if self.bar02.read():
                return {"depth": self.bar02.depth()}
            return {}
        except:
            return {}

    async def poll_sensors(self) -> dict:
        readings = []

        readings.append(self.get_depth())
        readings.append({"quaternion": [1.0, 0.0, 0.0, 0.0]})
        readings.append({"accelerometer": [0.0, 0.0, 0.0]})

        thrusters = {}
        motors = {}
        print(self.pwms)
        
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
        
        # TODO time-based translation instead of super small scalar to avoid framerate dependency
        self.gantry["x"] += (motors["gantry_left"] - motors["gantry_right"])/2 * 0.0003
        self.gantry["y"] += (motors["gantry_left"] + motors["gantry_right"])/2 * 0.0001
        self.gantry["x"] = max(-0.77, min(self.gantry["x"], 0.77))
        self.gantry["y"] = max(-0.17, min(self.gantry["y"], 0.17))
        self.arm_angle = (self.arm_angle + motors["buoyancy_arm"] * 0.04 )% 360
        # TODO implement retrieving from simulation
        # kinda being done
        
        readings.append({"thrusters": thrusters})
        readings.append({"motors": motors})
        readings.append({"gantry": self.gantry})
        readings.append({"arm_angle": self.arm_angle})
        
        readings_dict = {}
        for reading in readings:
            readings_dict.update(reading)

        return readings_dict
