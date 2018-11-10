import time
import pigpio # Remember to enable pigpiod
import numpy as np
from collections import deque
from scipy import stats
from IMU.BNO055 import BNO055
from IMU.ComplementaryFilter import ComplementaryFilter
from Controllers.PIDController import PIDController
from MotorAndEncoder.motor import motor
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
# Pin3: SDA
# Pin5: SCL

# Initialize Pi
pi = pigpio.pi()

# Initialize IMU
bno_rst_pin = 5
bno_address = 0x28
bno = BNO055(pi=pi, rst=5, address=0x28, bus=1)

'''
mpu_address = 0x68
mpu = mpu6050(mpu_address) # (Power with 3.3 V)
mpu.set_gyro_range(mpu.GYRO_RANGE_250DEG)
mpu.calibrate([-1.151972856280762+0.014508260794289421+0.012054316332367384+0.006829248829236366+0.0028514184745767985-0.009651640363941909, -0.421050890585, -0.603576335878, 0.482799144476, 1.09810059143, 0.482799])
'''

# Initialize Motors
motor2 = motor(pi,BI1,BI2,pwmB, encoder=False)
motor1 = motor(pi,AI1,AI2,pwmA, encoder=False)


# Initialize Controller
controller = PIDController(15,0,25)

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
#    count = 0
    period = 0.01
#
    t = time.time()
    current_time = time.time()
    time_start = current_time

    # Print system status and self test result.
    status, self_test, error = bno.get_system_status()
    print('System status: {0}'.format(status))
    print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
    # Print out an error if system status is in error mode.
    if status == 0x01:
        print('System error: {0}'.format(error))
        print('See datasheet section 4.3.59 for the meaning.')

    # Print BNO055 software revision and other diagnostic data.
    sw, bl, accel, mag, gyro = bno.get_revision()
    print('Software version:   {0}'.format(sw))
    print('Bootloader version: {0}'.format(bl))
    print('Accelerometer ID:   0x{0:02X}'.format(accel))
    print('Magnetometer ID:    0x{0:02X}'.format(mag))
    print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

    input('Press enter to begin reading BalanceBot, press Ctrl-C to quit...')
    while True:
        # Timing
        previous_time = current_time
        current_time = time.time()
        t = t + period

        # Get data from IMU
        gx, gy, gz = bno.read_gyroscope()
        ax, ay, az = bno.read_accelerometer()
        heading, roll, pitch = bno.read_euler()

        # Update Data Filter
        cfilt.update([gx,gy,gz,ax,ay,az], period)

        # Update Controller
        controller.update(0-cfilt.rollangle, period) #val_ref - val

        # Send Control Signal to Motors
        u1 = controller.get_u()
        u2 = u1
        motor1.set_duty_cycle(u1)
        motor2.set_duty_cycle(u2)
        print(pitch)

        # Sleep for Remaining Loop Time
        time.sleep(max(0, t-time.time()))

except KeyboardInterrupt:
    print('Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)



except IOError:
    print('IOERROR: Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)
#
