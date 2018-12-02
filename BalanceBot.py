import time
import pigpio # Remember to enable pigpiod
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import deque
from scipy import stats
from IMU.IMU import IMU
from IMU.BNO055 import BNO055
from IMU.MPU6050 import MPU6050
from IMU.ComplementaryFilter import ComplementaryFilter
from Controllers.PIDController import PIDController
from MotorAndEncoder.motor import motor
from MotorAndEncoder.rotary_encoder import rotary_encoder


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

# IMU Pins
#SDA: 2
#SCL: 3
mpu_vio = 4

# Initialize Pi
pi = pigpio.pi()

# Initialize IMU
#bno = BNO055(pi=pi) #, rst=bno_rst)
#imu = IMU(pi=pi, rst=5)
mpu = MPU6050(0x68, mpu_vio)


# Initialize Motors and Encoders
motor2 = motor(pi,BI1,BI2,pwmB,enc2A,enc2B, encoder=True)
motor1 = motor(pi,AI1,AI2,pwmA,enc1A,enc1B, encoder=True)
#enc1 = rotary_encoder(pi,enc1A,enc1B,countsPerRevolution=12)
#enc2 = rotary_encoder(pi,enc2A,enc2B,countsPerRevolution=12)


# Initialize Controllers
periodConv = 0.01/0.0021
K = 10
Ti = 30000
Td = 11
balance_controller = PIDController(K,K/Ti,K*Td)#K/Ti,K*Td)
#balance_controller = PIDController(14,0.175*periodConv,15/periodConv)

M1_controller = PIDController(125,10,0)
M2_controller = PIDController(125,10,0)


# Initialize Filter
cfilt = ComplementaryFilter(alpha=0.95, rollangle=0, angleOffset=0.56576161028555327)#1.0570298272571372)


# Initialize Data Storage Lists
omega_raw = []
theta_raw = []
u_raw = []
omega_kalman = []
theta_kalman = []
sim_time = []

# Run main loop
try:

    input('Press enter to begin running BalanceBot, press Ctrl-C to quit...')

    M1_pos = motor1.get_pos()
    M2_pos = motor2.get_pos()

    M1_diff_deck = deque([0, 0], 50)
    M2_diff_deck = deque([0, 0], 50)
    print(len(M1_diff_deck))

    #M1_pos_deck.append(M1_pos)
    #M2_pos_deck.append(M2_pos)

    period = 0.0021
    t = time.time()
    current_time = t

    r = 0
    count= 0

    time_start = time.time()
    while True:
        count = count + 1

        ## Do Timing
        previous_time = current_time
        current_time = time.time()
        t = t + period

        ## Get Data
        # Raw Data
        gx, gy, gz, _, ax, ay, az = mpu.get_all_data()

        M1_pos_prev = M1_pos
        M2_pos_prev = M2_pos
        M1_pos = motor1.get_pos()
        M2_pos = motor2.get_pos()

        M1_diff_deck.append(M1_pos)
        M2_diff_deck.append(M2_pos)


        RPS_M1 = (M1_diff_deck[-1] - M1_diff_deck[0])/ ((float(len(M1_diff_deck))-1)*period) #(M1_pos - M1_pos_prev)/(period) #np.mean(M1_diff_deck)/period
        RPS_M2 = (M2_diff_deck[-1] - M2_diff_deck[0])/ ((float(len(M2_diff_deck))-1)*period) #(M1_pos - M1_pos_prev)/(period)

        ## Update Data Filter
        cfilt.update([gx,gy,gz,ax,ay,az], current_time-previous_time)
        #if abs(cfilt.rollangle) > 35:
        #    break

        ## Update Control Signals
        balance_controller.update((r-cfilt.rollangle)) #val_ref - val
        M1_controller.update( 100/60 - RPS_M1) #balance_controller.u - RPM_M1)
        M2_controller.update( 100/60 - RPS_M2) #balance_controller.u - RPM_M2)

        # Send Control Signal to Motors
        motor1.set_duty_cycle(M1_controller.u)
        motor2.set_duty_cycle(M2_controller.u)


        if (count % 20) == 0:
            #print(M1_pos)
            print("ERROR RPM_M1: {:<+10.2F}, ERROR RPM_M2: {:<+10.2F}".format( 100 - RPS_M1*60, 100 - RPS_M2*60))
            #print("theta: {:+4.2F}, uP: {:+8.3F}, uI: {:+8.3F} ud: {:+8.3F} u: {:+8.2F}".format(cfilt.rollangle, balance_controller.eP*balance_controller.kP, balance_controller.eI*balance_controller.kI, balance_controller.eD*balance_controller.kD, balance_controller.u))
            #print('Theta: {0:.1f},  u: {1:.1f}'.format(cfilt.rollangle,balance_controller.u))
            #print('gx: {}, gy: {}, gz: {}, ax: {}, ay: {}, az: {}'.format(gx,gy,gz,ax,ay,az))
            #print('PER: {}'.format((current_time-time_start)/count))
            pass
        #if (count % 25) == 0:
        #    if r-theta > 0:
        #        print("\033[92m theta: {0: 0.3F} er: {1: 0.3F} up: {2: 0.3F}, uI: {3: 0.3F}, uD: {4: 0.3F} \033[0m".format(theta, controller.eP, controller.eP*controller.kP, controller.eI*controller.kI, controller.eD*controller.kD))
        #    else:
        #        print("\033[91m theta: {0: 0.3F} er: {1: 0.3F} up: {2: 0.3F}, uI: {3: 0.3F}, uD: {4: 0.3F} \033[0m".format(theta, controller.eP, controller.eP*controller.kP, controller.eI*controller.kI, controller.eD*controller.kD))

        #omega_raw.append((cfilt.rollangle,0))
        theta_raw.append(cfilt.rollangle)
        u_raw.append(balance_controller.u)
        sim_time.append(current_time)
        # Sleep for Remaining Loop Time
        time.sleep(max(0, t-time.time()))


    #print(tuple(np.mean(omega_raw,axis=0)))

    print('PER: {}'.format((time.time() - time_start)/count))
    #imu.stop_imu()
    print('Angle too high, turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)


    plt.figure(0)
    plt.plot(np.array(sim_time)-sim_time[0],np.array(theta_raw),label='theta')
    plt.plot(np.array(sim_time)-sim_time[0],np.array(u_raw),label='u')
    #plt.title('rpm v time')
    plt.xlabel('Time [s]')
    plt.ylabel('Magnitude')
    plt.legend()
    plt.grid(True)
    plt.show()



except KeyboardInterrupt:
    #imu.stop_imu()
    print('PER: {}'.format((time.time() - time_start)/count))
    print('Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)



except IOError:
    #imu.stop_imu()
    print('IOERROR: Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)
