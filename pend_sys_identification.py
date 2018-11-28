import time
import pigpio # Remember to enable pigpiod
import numpy as np
from collections import deque
from MotorAndEncoder.motor import motor
from MotorAndEncoder.rotary_encoder import rotary_encoder

"""
This script is meant to help identify system parameters for the full pendulum
system. The pendulum should start at 90 degrees with the wheels held stationary.
The program will wait one second, apply 12 volts, then end once the pendulum
becomes vertical, which is determined through the encoder readings.
"""

# Define pin numbers:
AI1 = 14
AI2 = 15
pwmA = 18

BI1 = 24
BI2 = 23
pwmB = 19

enc1A = 7
enc1B = 8
enc2A = 9
enc2B = 11

# Initialize Pi
pi = pigpio.pi()

# Initialize Motors and Encoders
motor1 = motor(pi,AI1,AI2,pwmA,dec_pin1=enc1A,dec_pin2=enc1B,countsPerRevolution=46.85*12,encoder=True)
motor2 = motor(pi,BI1,BI2,pwmB,dec_pin1=enc1B,dec_pin2=enc1B,countsPerRevolution=46.85*12,encoder=True)

v_input = 12
dc = v_input/12*100

motor1.set_duty_cycle(0)
motor2.set_duty_cycle(0)
start_time = time.time()
t = 0
pos1 = motor1.get_pos()
pos2 = motor2.get_pos()
output_vec = []
while pos1 < 90 and pos2 < 90:
    # Collect data for ten seconds
    t_prev = t
    t = time.time()-start_time

    pos1 = motor1.get_pos()
    pos2 = motor2.get_pos()

    output_vec.append('{},{},{}\n'.format(t, pos1, pos2))

    if t > 1:
        motor1.set_duty_cycle(dc)
        motor2.set_duty_cycle(dc)

#print(pos1)
motor1.set_duty_cycle(0)

# Save data to file:
f = open('pend_data.txt','w')
for line in output_vec:
    f.write(line)
f.close()
