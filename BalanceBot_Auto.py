import time
import pigpio # Remember to enable pigpiod
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import deque
#from scipy import stats
from IMU.IMU import IMU
from IMU.BNO055 import BNO055
from IMU.MPU6050 import MPU6050
from IMU.ComplementaryFilter import ComplementaryFilter
from IMU.MadgwickAHRS import MadgwickAHRS
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

# LED Pins to turn on
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
#bno = BNO055(pi=pi) #, rst=bno_rst)
#imu = IMU(pi=pi, rst=5)
mpu = MPU6050(0x68, mpu_vio)


# Initialize Motors and Encoders
motor2 = motor(pi,BI1,BI2,pwmB,enc2A,enc2B, encoder=True)
motor1 = motor(pi,AI1,AI2,pwmA,enc1A,enc1B, encoder=True)
#enc1 = rotary_encoder(pi,enc1A,enc1B,countsPerRevolution=12)
#enc2 = rotary_encoder(pi,enc2A,enc2B,countsPerRevolution=12)


# Initialize Controllers
period = 0.0021

#balance_controller = PIDController(10,0.21,55) #Worked on carpet
#balance_controller = PIDController(10,0.21,65) # Worked with adjusted offset, large oscillations
#balance_controller = PIDController(10,0.21,67) # Worked with adjusted offset, some Kd jitter
balance_controller = PIDController(10,0.225,63) # Worked with adjusted offset aimed at reducing x oscillations

# Initialize Filter
cfilt = ComplementaryFilter(alpha=0.98, rollangle=0, angleOffset=1.1359977682671618*0.59)#.56576161028555327/2)#1.0570298272571372)

# Initialize Data Storage Lists
omega_raw = []
#theta_raw = []
#u_raw = []
#omega_kalman = []
#theta_kalman = []
#sim_time = []

# Run main loop
try:
    #Commented out because this is the auto run version
    #input('Press enter to begin running BalanceBot, press Ctrl-C to quit...')

    # LED Flashing to Warn that Program is about to running (Blue is 1 and Red is 2)
    led_wait = 0.25
    pi.write(led1, 1)       #Blue on
    time.sleep(led_wait)
    pi.write(led1, 0)       #Blue off
    time.sleep(led_wait)
    pi.write(led1, 1)       #Blue on
    time.sleep(led_wait)
    pi.write(led1, 0)       #Blue off
    time.sleep(led_wait)
    pi.write(led2, 1)       #Red on
    time.sleep(led_wait)
    pi.write(led2, 0)       #Red off
    time.sleep(led_wait)
    pi.write(led2, 1)       #Red on
    time.sleep(led_wait)
    pi.write(led2, 0)       #Red off
    time.sleep(led_wait)


    M1_pos = motor1.get_pos()
    M2_pos = motor2.get_pos()

    M1_diff_deck = deque([0, 0], 25)
    M2_diff_deck = deque([0, 0], 25)

    t = time.time()
    current_time = t

    r = 0
    count= 0
    xref = 0

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

        #M1_pos_prev = M1_pos
        #M2_pos_prev = M2_pos
        M1_pos = motor1.get_pos()
        M2_pos = motor2.get_pos()

        M1_diff_deck.append(M1_pos)
        M2_diff_deck.append(M2_pos)


        #RPS_M1 = (M1_diff_deck[-1] - M1_diff_deck[0])/ ((float(len(M1_diff_deck))-1)*period) #(M1_pos - M1_pos_prev)/(period) #np.mean(M1_diff_deck)/period
        #RPS_M2 = (M2_diff_deck[-1] - M2_diff_deck[0])/ ((float(len(M2_diff_deck))-1)*period) #(M1_pos - M1_pos_prev)/(period)

        ## Update Data Filter
        cfilt.update([gx,gy,gz,ax,ay,az], current_time-previous_time)
        if abs(cfilt.rollangle) > 35:
           break
        #omega_raw.append((cfilt.accel_angle,gx,gy,gz))

        ## Update Control Signals
        x = ((np.mean(M1_diff_deck)+np.mean(M2_diff_deck))/2)*10
        error = (r-cfilt.rollangle) # + 0.07*min([(x - xref), 2.5])
        balance_controller.update(error)#-0.1*motor1.get_pos())
        #M1_controller.update( balance_controller.u - RPS_M1)
        #M2_controller.update( balance_controller.u - RPS_M2)

        #M1_controller.update( balance_controller.u - RPS_M1) #160/60 - RPS_M1)
        #M2_controller.update( balance_controller.u - RPS_M2) #160/60 - RPS_M2)
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
            #print('{:>6.3F}'.format(RPS_M1*60))
            #print("ERROR RPM_M1: {:<+10.2F}, ERROR RPM_M2: {:<+10.2F}".format( 100 - RPS_M1*60, 100 - RPS_M2*60))
            print("count:: {:d}  :::: theta: {: >+7.2F}, uP: {: >+7.2F}, uI: {: >+7.2F}, ud: {: >+6.1F}, u: {: >+6.1F}, X: {: >+6.1F}".format(count, cfilt.rollangle, balance_controller.eP*balance_controller.kP, balance_controller.eI*balance_controller.kI, balance_controller.eD*balance_controller.kD, balance_controller.u, x))
            #print('Theta: {0:.1f},  u: {1:.1f}'.format(cfilt.rollangle,balance_controller.u))
            #print('Accel: {: >+7.2F}, Gyro: {: >+7.2F}, Pitch: {: >+7.2F}, gx: {: >+7.2F},, gy: {: >+7.2F},, gz: {: >+7.2F},, ax: {: >+7.2F},, ay: {: >+7.2F},, az: {: >+7.2F},'.format(cfilt.accel_angle,cfilt.gyro_angle,cfilt.rollangle,gx,gy,gz,ax,ay,az))
            #print('PER: {}'.format((current_time-time_start)/count))
            pass
        #if (count % 25) == 0:
        #    if r-theta > 0:
        #        print("\033[92m theta: {0: 0.3F} er: {1: 0.3F} up: {2: 0.3F}, uI: {3: 0.3F}, uD: {4: 0.3F} \033[0m".format(theta, controller.eP, controller.eP*controller.kP, controller.eI*controller.kI, controller.eD*controller.kD))
        #    else:
        #        print("\033[91m theta: {0: 0.3F} er: {1: 0.3F} up: {2: 0.3F}, uI: {3: 0.3F}, uD: {4: 0.3F} \033[0m".format(theta, controller.eP, controller.eP*controller.kP, controller.eI*controller.kI, controller.eD*controller.kD))

        #omega_raw.append((cfilt.rollangle,0))
        #theta_raw.append(cfilt.rollangle)
        #u_raw.append(balance_controller.u)
        #sim_time.append(current_time)
        # Sleep for Remaining Loop Time
        time.sleep(max(0, t-time.time()))


    #print(tuple(np.mean(omega_raw,axis=0)))

    print('PER: {}'.format((time.time() - time_start)/count))
    #print(np.mean(theta_raw))
    #imu.stop_imu()
    print('Angle too high, turning off motors: {}'.format(cfilt.rollangle))
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)


    plt.figure(0)
    plt.rcParams.update({'font.size': 22})
    plt.plot(np.array(sim_time)-sim_time[0],np.array(theta_raw),label='theta')
    plt.plot(np.array(sim_time)-sim_time[0],np.array(u_raw),label='u')
    plt.title('BalanceBot Trial: Kp:{: <4.2F}, Ki:{: <4.2F}, Kd:{: <4.2F}'.format(balance_controller.kP,balance_controller.kI,balance_controller.kD))
    plt.xlabel('Time [s]')
    plt.ylabel('Magnitude')
    plt.legend()
    plt.grid(True)
    plt.savefig('BB_run_.png'.format(1))
    #plt.show()



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
