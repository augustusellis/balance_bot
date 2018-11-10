import time
import pigpio # Remember to enable pigpiod
import numpy as np
from collections import deque
from scipy import stats
from IMU.IMU import IMU
from IMU.BNO055 import BNO055
from IMU.ComplementaryFilter import ComplementaryFilter
from Controllers.PIDController import PIDController
from MotorAndEncoder.motor import motor
from MotorAndEncoder.rotary_encoder import rotary_encoder

#
## Moving Average
#moving_average_length = 2000
#errorDeque = deque([], moving_average_length)
#uDeque = deque([], moving_average_length)
#'''
#GxDeq = deque([], moving_average_length)
#GyDeq = deque([], moving_average_length)
#GzDeq = deque([], moving_average_length)
#AxDeq = deque([], moving_average_length)
#AyDeq = deque([], moving_average_length)
#AzDeq = deque([], moving_average_length)
#'''
# Define pin numbers:
AI1 = 14
AI2 = 15
pwmA = 18

BI1 = 24
BI2 = 23
pwmB = 19

enc1A = 1 # CHANGE THESE BEFORE RUNNING ENCODERS
enc1B = 1
enc2A = 1
enc2B = 1
# Pin3: SDA
# Pin5: SCL

# Initialize Pi
pi = pigpio.pi()

# Initialize IMU
#bno = BNO055(pi=pi, rst=5)
imu = IMU(pi=pi, rst=5)


# Initialize Motors and Encoders
motor2 = motor(pi,BI1,BI2,pwmB, encoder=False)
motor1 = motor(pi,AI1,AI2,pwmA, encoder=False)
#enc1 = rotary_encoder(pi,enc1A,enc1B,countsPerRevolution=12)
#enc2 = rotary_encoder(pi,enc2A,enc2B,countsPerRevolution=12)


# Initialize Controller
controller = PIDController(5,0,10)

# Initialize Filter
'''
Q = 0.001
R = 250*0.05
kalman = KalmanFilter(np.array([[0]]), np.array([[1]]), np.array([[Q]]), np.array([[R]]))
'''
cfilt = ComplementaryFilter(alpha=0.98, rollangle=0)

# Initialize Data Storage Lists
omega_raw = []
theta_raw = []
omega_kalman = []
theta_kalman = []
sim_time = []

# Run main loop
try:

    # Print system status and self test result.
    status, self_test, error = imu.bno.get_system_status()
    print('System status: {0}'.format(status))
    print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
    # Print out an error if system status is in error mode.
    if status == 0x01:
        print('System error: {0}'.format(error))
        print('See datasheet section 4.3.59 for the meaning.')


    input('Press enter to begin running BalanceBot, press Ctrl-C to quit...')
    period = 0.01
    t = time.time()
    current_time = t
    imu.start_imu()

    r = 0
    theta_offset = 2.1
    while True:
        # Timing
        previous_time = current_time
        current_time = time.time()
        t = t + period

        '''
        # Get data from IMU
        gx, gy, gz = bno.read_gyroscope()
        ax, ay, az = bno.read_accelerometer()
        heading, roll, pitch = bno.read_euler()

        # Update Data Filter
        cfilt.update([gx,gy,gz,ax,ay,az], period)
        '''

        # Update Controller
        theta = imu.theta_eul+theta_offset
        controller.update(r-theta, period) #val_ref - val

        # Send Control Signal to Motors
        u1 = controller.get_u()
        u2 = u1
        motor1.set_duty_cycle(u1)
        motor2.set_duty_cycle(u2)
        print("theta: {}".format(theta))

        # Sleep for Remaining Loop Time
        time.sleep(max(0, t-time.time()))


except KeyboardInterrupt:
    imu.stop_imu()
    print('Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)


except IOError:
    imu.stop_imu()
    print('IOERROR: Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)
