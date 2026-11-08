import os
import sys

# rov_config lives with the surface code so pin changes only happen in one place
sys.path.append(os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../surface/core")
))

from rov_config import thruster_config
from rov_config import motor_config


# Shared by the physical and simulated ROVs. Subclasses provide set_pin_pwm and
# must record each commanded value in self.pwms.
class ROVBase:
    def __init__(self):
        self.pwms = {}
        self.gantry = {"x": 0.0, "y": 0.0}
        self.arm_angle = 0.0

    def stop_all(self) -> bool:
        failures = {}
        for number in self.pwms:
            try:
                self.set_pin_pwm(number, 1500)
            except Exception as e:
                failures[number] = e

        if failures:
            print(f'FAILED TO STOP PINS: {failures!r}')
        return not failures

    async def flush_pin_pwms(self):
        pass

    def actuator_readings(self) -> dict:
        thrusters = {
            t["name"]: (self.pwms.get(t["pin"], 1500) - 1500) * t.get("direction", 1) * t.get("handing", 1)
            for t in thruster_config
        }
        motors = {
            m["name"]: (self.pwms.get(m["pin"], 1500) - 1500) * m.get("direction", 1)
            for m in motor_config
        }

        # TODO time-based translation instead of super small scalar to avoid framerate dependency
        self.gantry["x"] += (motors["gantry_left"] - motors["gantry_right"])/2 * 0.0003
        self.gantry["y"] += (motors["gantry_left"] + motors["gantry_right"])/2 * 0.0001
        self.gantry["x"] = max(-0.77, min(self.gantry["x"], 0.77))
        self.gantry["y"] = max(-0.17, min(self.gantry["y"], 0.17))
        self.arm_angle = (self.arm_angle + motors["buoyancy_arm"] * 0.04 )% 360

        return {
            "thrusters": thrusters,
            "motors": motors,
            "gantry": self.gantry,
            "arm_angle": self.arm_angle,
        }
