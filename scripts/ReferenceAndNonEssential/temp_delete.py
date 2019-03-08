import time
import pigpio # Remember to enable pigpiod
import numpy as np
from math import copysign
from collections import deque
from IMU.MPU6050 import MPU6050
from IMU.ComplementaryFilter import ComplementaryFilter
from Controllers.PIDController import PIDController
from MotorAndEncoder.motor import motor


# Define pin numbers:

# IMU Pins
#SDA: 2
#SCL: 3
mpu_vio = 4

# Initialize IMU
mpu = MPU6050(0x68, mpu_vio)

# Run main loop
try:
    #Commented out because this is the auto run version
    #input('Press enter to begin running BalanceBot, press Ctrl-C to quit...')


    r = 0
    count= 0
    xref = 0

    t = time.time()
    current_time = t
    time_start = t
    while True:
        count = count + 1

        # Get Data
        gx, gy, gz, _, ax, ay, az = mpu.get_all_data()

        M1_pos = motor1.get_pos()
        M2_pos = motor2.get_pos()

        M1_pos_deck.append(M1_pos)
        M2_pos_deck.append(M2_pos)

        # Send Control Signal to Motors
        motor1.set_duty_cycle(balance_controller.u)
        motor2.set_duty_cycle(balance_controller.u)

