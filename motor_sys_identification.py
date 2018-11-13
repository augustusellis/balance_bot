import time
import pigpio # Remember to enable pigpiod
import numpy as np
from collections import deque
from MotorAndEncoder.motor import motor
from MotorAndEncoder.rotary_encoder import rotary_encoder

"""
This script is meant to help identify system parameters for the DC motor by
saving time and RPM data for a given input voltage.
"""

# Define pin numbers:
AI1 = 14
AI2 = 15
pwmA = 18

enc1A = 7 # CHANGE THESE BEFORE RUNNING ENCODERS
enc1B = 8


# Initialize Pi
pi = pigpio.pi()

# Initialize Motors and Encoders
motor1 = motor(pi,AI1,AI2,pwmA,dec_pin1=enc1A,dec_pin2=enc1B,countsPerRevolution=46.85*12,encoder=True)

v_input = 12
dc = v_input/14.05*100

motor1.set_duty_cycle(0)
start_time = time.time()
t = 0
pos = motor1.get_pos()
output_vec = []
while t < 10:
    # Collect data for ten seconds
    t_prev = t
    t = time.time-start_time()

    pos_prev = pos
    pos = motor1.get_pos()

    output_vec.append('{},{}'.format(t, (pos-pos_prev)/(t-t_prev)))

    if t > 1:
        motor1.set_duty_cycle(dc)


# Save data to file:
f = open('rpm_data.txt','w')
for line in output_vec:
    f.write(line)
f.close()
