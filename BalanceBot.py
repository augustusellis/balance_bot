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


# Initialize Controller
periodConv = 0.01/0.0021
K = 10
Ti = 30000
Td = 11
controller = PIDController(K,K/Ti,K*Td)#K/Ti,K*Td)
#controller = PIDController(14,0.175*periodConv,15/periodConv)


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

    # Print system status and self test result.
    #status, self_test, error = bno.get_system_status()
    #print('System status: {0}'.format(status))
    #print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
    ## Print out an error if system status is in error mode.
    #if status == 0x01:
    #   print('System error: {0}'.format(error))
    #    print('See datasheet section 4.3.59 for the meaning.')


    input('Press enter to begin running BalanceBot, press Ctrl-C to quit...')

    period = 0.0021
    imu_period = 0.01
    imu_read_time = time.time();
    t = time.time()
    current_time = t
    #imu.start_imu()

    r = 0
    count= 0
    time_start = time.time()

    theta = 0
    gx = 0
    while True:
        count = count + 1
        # Do Timing
        previous_time = current_time
        current_time = time.time()
        #if current_time-previous_time>period:
        #    print('\033[91m WARNING: TOO SLOW \033[0m')
        t = t + period

        gx, gy, gz, _, ax, ay, az = mpu.get_all_data()
        # Get data from IMU
        #if True: #(current_time - imu_read_time) >= imu_period:
            # Get New Data
            #gx, gy, gz = bno.read_gyroscope()
            #ax, ay, az = bno.read_accelerometer()
            #heading, roll, pitch = bno.read_euler()
            #theta = pitch + theta_offset
            #imu_time_prev = imu_read_time
            #imu_read_time = current_time
        #else:
            # Linear Extrapolation of Data
            #theta = theta + gx*(current_time-previous_time)
        # Update Data Filter
        cfilt.update([gx,gy,gz,ax,ay,az], current_time-previous_time)

        if abs(cfilt.rollangle) > 35:
            break

        #theta = 0
        # Update Controller
        controller.update((r-cfilt.rollangle)) #val_ref - val

        # Send Control Signal to Motors
        #u1 = controller.u
        #u2 = u1
        #r_pos = 46.85*12 # One Revolution
        #k = 1
        #u1 = k*(r_pos-motor1.get_pos())
        #u2 = k*(r_pos-motor2.get_pos())
        motor1.set_duty_cycle(controller.u)
        motor2.set_duty_cycle(controller.u)


        if (count % 1) == 0:
            print("theta: {:+4.2F}, uP: {:+8.3F}, uI: {:+8.3F} ud: {:+8.3F} u: {:+8.2F}".format(cfilt.rollangle, controller.eP*controller.kP, controller.eI*controller.kI, controller.eD*controller.kD, controller.u))
            #print('Theta: {0:.1f},  u: {1:.1f}'.format(cfilt.rollangle,controller.u))
            #print('gx: {}, gy: {}, gz: {}, ax: {}, ay: {}, az: {}'.format(gx,gy,gz,ax,ay,az))
            #print('PER: {}'.format((current_time-time_start)/count))

        #if (count % 25) == 0:
        #    if r-theta > 0:
        #        print("\033[92m theta: {0: 0.3F} er: {1: 0.3F} up: {2: 0.3F}, uI: {3: 0.3F}, uD: {4: 0.3F} \033[0m".format(theta, controller.eP, controller.eP*controller.kP, controller.eI*controller.kI, controller.eD*controller.kD))
        #    else:
        #        print("\033[91m theta: {0: 0.3F} er: {1: 0.3F} up: {2: 0.3F}, uI: {3: 0.3F}, uD: {4: 0.3F} \033[0m".format(theta, controller.eP, controller.eP*controller.kP, controller.eI*controller.kI, controller.eD*controller.kD))

        #omega_raw.append((cfilt.rollangle,0))
        theta_raw.append(cfilt.rollangle)
        u_raw.append(controller.u)
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
