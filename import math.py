import math
import pygame

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

joy_x = joystick.get_axis(0)
joy_y = -joystick.get_axis(1)

def get_motor_vectors(joy_x, joy_y, deadzone=0.1, max_speed=100):
    #Apply Deadzone
    if abs(joy_x) < deadzone: joy_x = 0
    if abs(joy_y) < deadzone: joy_y = 0

    #Keep diagonal speed same as straight
    magnitude = math.sqrt(joy_x**2 + joy_y**2)
    if magnitude > 1.0:
        joy_x /= magnitude
        joy_y /= magnitude

    #CoreXY Transformation + adding the vectors
    motor_a = (joy_x + joy_y) * max_speed
    motor_b = (joy_x - joy_y) * max_speed

    return motor_a, motor_b