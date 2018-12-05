import time
import pigpio # Remember to enable pigpiod
import numpy as np
from math import copysign
from collections import deque
from IMU.MPU6050 import MPU6050
from IMU.ComplementaryFilter import ComplementaryFilter
from Controllers.PIDController import PIDController
from MotorAndEncoder.motor import motor
#from MotorAndEncoder.rotary_encoder import rotary_encoder


# Define pin numbers:
# Motor and Encoder Pins
AI1 = 14
AI2 = 15
pwmA = 18

BI1 = 23
BI2 = 24
pwmB = 19

enc1A = 7
enc1B = 8
enc2A = 11
enc2B = 9

# Directional Indicator LED Pins to turn on
led1 = 5
led2 = 6

# IMU Pins
#SDA: 2
#SCL: 3
mpu_vio = 4

# Initialize Pi
pi = pigpio.pi()

# Initializing LED Pins to output mode
pi.set_mode(led1, pigpio.OUTPUT)
pi.write(led1, 0)
pi.set_mode(led1, pigpio.OUTPUT)
pi.write(led2, 0)

# Initialize IMU
mpu = MPU6050(0x68, mpu_vio)


# Initialize Motors and Encoders
motor2 = motor(pi,BI1,BI2,pwmB,enc2A,enc2B, encoder=True)
motor1 = motor(pi,AI1,AI2,pwmA,enc1A,enc1B, encoder=True)

# Initialize Controllers
period = 0.0021

#balance_controller = PIDController(10,0.21,55) # Worked on carpet
#balance_controller = PIDController(10,0.21,65) # Worked with adjusted offset, large oscillations
#balance_controller = PIDController(10,0.21,67) # Worked with adjusted offset, some Kd jitter
balance_controller = PIDController(10,0.225,63) # Worked with adjusted offset aimed at reducing x oscillations

# Initialize Filter
cfilt = ComplementaryFilter(alpha=0.98, rollangle=0, angleOffset=0.730238683)#.56576161028555327/2)#1.0570298272571372)

# Run main loop
try:

    input('Press enter to begin running BalanceBot, press Ctrl-C to quit...')

    M1_pos = motor1.get_pos()
    M2_pos = motor2.get_pos()

    M1_pos_deck = deque([0, 0], 25)
    M2_pos_deck = deque([0, 0], 25)


    r = 0
    count= 0
    xref = 0

    t = time.time()
    current_time = t
    time_start = t
    while True:
        count = count + 1

        # Do Timing
        previous_time = current_time
        current_time = time.time()
        t = t + period

        # Get Data
        gx, gy, gz, _, ax, ay, az = mpu.get_all_data()

        M1_pos = motor1.get_pos()
        M2_pos = motor2.get_pos()

        M1_pos_deck.append(M1_pos)
        M2_pos_deck.append(M2_pos)

        # Update Data Filter
        cfilt.update([gx,gy,gz,ax,ay,az], current_time-previous_time)
        if abs(cfilt.rollangle) > 35:
            print('Angle too high, Breaking Loop: {}'.format(cfilt.rollangle))
            break

        # Update Control Signals
        x = ((np.mean(M1_pos_deck)+np.mean(M2_pos_deck))/2)*10
        error = (r-cfilt.rollangle) + 0.025*copysign(1,x-xref)*min([abs(x - xref), 2.5])
        balance_controller.update(error)

        # Send Control Signal to Motors
        motor1.set_duty_cycle(balance_controller.u)
        motor2.set_duty_cycle(balance_controller.u)


        # Check angle and turn on correct LED
        if cfilt.rollangle > 0:
            pi.write(led1, 1)
            pi.write(led2, 0)
        else:
            pi.write(led1, 0)
            pi.write(led2, 1)

        if (count % 40) == 0:
            #print("count:: {:d}  :::: theta: {: >+7.2F}, uP: {: >+7.2F}, uI: {: >+7.2F}, ud: {: >+6.1F}, u: {: >+6.1F}, X: {: >+6.1F}".format(count, cfilt.rollangle, balance_controller.eP*balance_controller.kP, balance_controller.eI*balance_controller.kI, balance_controller.eD*balance_controller.kD, balance_controller.u, x))
            #print('Accel: {: >+7.2F}, Gyro: {: >+7.2F}, Pitch: {: >+7.2F}, gx: {: >+7.2F},, gy: {: >+7.2F},, gz: {: >+7.2F},, ax: {: >+7.2F},, ay: {: >+7.2F},, az: {: >+7.2F},'.format(cfilt.accel_angle,cfilt.gyro_angle,cfilt.rollangle,gx,gy,gz,ax,ay,az))
            pass

        # Sleep for Remaining Loop Time
        time.sleep(max(0, t-time.time()))


    print('PER: {}'.format((time.time() - time_start)/count))
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)
    pi.write(led1, 0)
    pi.write(led2, 0)


except KeyboardInterrupt:
    print('PER: {}'.format((time.time() - time_start)/count))
    print('Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)
    pi.write(led1, 0)
    pi.write(led2, 0)



except IOError:
    print('IOERROR: Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)
    pi.write(led1, 0)
    pi.write(led2, 0)
