import time
from mpu6050 import mpu6050
from PIDController import PIDController
from madgwickahrs import MadgwickAHRS
from KalmanFilter import KalmanFilter
from scipy import stats
from motor import motor
import pigpio # Remember to enable pigpiod
import numpy as np
from collections import deque
import matplotlib.pyplot as plt

# Moving Average
moving_average_length = 2000
errorDeque = deque([], moving_average_length)
uDeque = deque([], moving_average_length)

GxDeq = deque([], moving_average_length)
GyDeq = deque([], moving_average_length)
GzDeq = deque([], moving_average_length)
AxDeq = deque([], moving_average_length)
AyDeq = deque([], moving_average_length)
AzDeq = deque([], moving_average_length)

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

# Initialize MPU
mpu_address = 0x68
mpu = mpu6050(mpu_address) # (Power with 3.3 V)
mpu.set_gyro_range(mpu.GYRO_RANGE_250DEG)
mpu.calibrate([-1.151972856280762+0.014508260794289421+0.012054316332367384+0.006829248829236366+0.0028514184745767985-0.009651640363941909, -0.421050890585, -0.603576335878, 0.482799144476, 1.09810059143, 0.482799])


# Initialize Motors
motor2 = motor(pi,BI1,BI2,pwmB, encoder=False)
motor1 = motor(pi,AI1,AI2,pwmA, encoder=False)


# Initialize Controller
controller = PIDController(15,0,25)

# Initialize Kalman Filter
#madgeFilter = MadgwickAHRS()
Q = 0.001
R = 250*0.05
kalman = KalmanFilter(np.array([[0]]), np.array([[1]]), np.array([[Q]]), np.array([[R]]))

omega_raw = []
theta_raw = []
omega_kalman = []
theta_kalman = []
sim_time = []
# Run main loop

try:
    theta_ref = 0
    #val_ref = 0.1 #9.55
    #yref = 0.45

    count = 0
    period = 0.0001

    theta = 0

    t = time.time()
    current_time = time.time()
    time_start = current_time

    theta_raw_val = 0
    theta_kalman_val = 0

    while count < moving_average_length: #True:
        previous_time = current_time
        current_time = time.time()

        t = t + period

        gyro_data = mpu.get_gyro_data()
        accel_data = mpu.get_accel_data()

        gyro_data = [gyro_data['x'], gyro_data['y'], gyro_data['z']]
        accel_data = [accel_data['x'], accel_data['y'], accel_data['z']]



        kalman.update(np.array([gyro_data[0]]), np.array([0]))

        theta_raw_val = theta_raw_val + gyro_data[0]*(current_time - previous_time)
        theta_kalman_val = theta_kalman_val + kalman.xk_hat[0][0]*(current_time - previous_time)

        omega_raw.append(gyro_data[0])
        omega_kalman.append(kalman.xk_hat[0][0])
        theta_raw.append(theta_raw_val)
        theta_kalman.append(theta_kalman_val)
        sim_time.append(current_time-time_start)
        #madgeFilter.update_imu(gyro_data, accel_data, current_time - previous_time)
        '''
        GxDeq.append(gyro_data[0])
        GyDeq.append(gyro_data[1])
        GzDeq.append(gyro_data[2])
        AxDeq.append(accel_data[0])
        AyDeq.append(accel_data[1])
        AzDeq.append(accel_data[2])
        '''
        count = count + 1
        val = (gyro_data[0])

        '''
        controller.update(theta - theta_ref, period) #val_ref - val

        u1 = controller.get_u()
        u2 = u1

        #print('{:+.4f}, {:+.4f}'.format(theta, u1))
        motor1.set_duty_cycle(u1)
        motor2.set_duty_cycle(u2)
        '''
        theta = theta + val*(current_time - previous_time)


        #errorDeque.append(accel_data['x'])
        #uDeque.append(u1)

        #if (count % moving_average_length) == 0:
        #   print('{:+.9f}, {:+.9f}'.format(np.mean(errorDeque), np.mean(uDeque)))
        #   pass

        #count = count + 1

        #print('{:+.4f}'.format(theta))
        #print(madgeFilter.quaternion._q)

        time.sleep(max(0, t-time.time()))


    plt.rcParams.update({'font.size': 22})
    plt.figure()
    plt.plot(sim_time, omega_raw, 'b*', label = 'Raw', linewidth=4.0)
    plt.plot(sim_time, omega_kalman, 'm', label = 'Kalman Filtered', linewidth=4.0)
    plt.xlabel('Time, [s]')
    plt.ylabel('Wx, [deg/s]')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(sim_time, theta_raw, 'b*', label = 'Raw', linewidth=4.0)
    plt.plot(sim_time, theta_kalman, 'm', label = 'Kalman Filtered', linewidth=4.0)
    plt.xlabel('Time, [s]')
    plt.ylabel('theta_x, [deg/s]')
    plt.legend()
    plt.grid()

    plt.show()

    slope, intercept, r_value, p_value, std_err = stats.linregress(sim_time,theta_raw)
    print('{}, {}, {}, {}, {}'.format(slope, intercept, r_value, p_value, std_err))

except KeyboardInterrupt:
    print('Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)


except IOError:
    print('IOERROR: Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)
