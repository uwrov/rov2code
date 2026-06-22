import busio
import board
import time
from .rov_config import thruster_config
from .rov_config import motor_config

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo, Device
Device.pin_factory = PiGPIOFactory()


class ROV:
    def __init__(self):
        self.servos = {}
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
        except Exception as e:
            print(e)
            print("UNABLE TO CONNECT TO IMU")
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
