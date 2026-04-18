import time
import busio
import board
import time
from gpiozero import Servo

class ROV:
    def __init__(self):
        self.servos = {}
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
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
        #TODO time-based translation instead of super small scalar to avoid framerate dependency
        self.gantry["x"] += (motors["gantry_left"] - motors["gantry_right"])/2 * 0.0003
        self.gantry["y"] += (motors["gantry_left"] + motors["gantry_right"])/2 * 0.0001
        self.gantry["x"] = max(-0.77, min(self.gantry["x"], 0.77))
        self.gantry["y"] = max(-0.17, min(self.gantry["y"], 0.17))
        self.arm_angle = (self.arm_angle + motors["buoyancy_arm"] * 0.04 )% 360
        
        # TODO implement retrieving from simulation
        # kinda being done
        return {
            "quaternion": gyroscope,       # match Godot expectation
            "accelerometer": accelerometer,
            "thrusters": thrusters,
            "motors": motors,
            "gantry" : self.gantry,
            "arm_angle" : self.arm_angle
        }
