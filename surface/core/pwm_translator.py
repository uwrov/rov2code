import numpy as np
##OLD --- LOGIC MOVED TO force_to_pwm.py
def f(x):
    if x < 0:
        return int(np.interp(x, [-1, 0], [1100, 1464]))
    elif x > 0:
        return int(np.interp(x, [0, 1], [1536, 1900]))
    else:
        return 1500
    
map_power_to_pwm = np.vectorize(f)

def convert_motor_powers_to_pwms(motor_powers):
    motor_powers = motor_powers
    return map_power_to_pwm(motor_powers).flatten().tolist()

