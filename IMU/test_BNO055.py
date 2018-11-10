import time
import pigpio
from BNO055 import BNO055
from ComplementaryFilter import ComplementaryFilter
import numpy as np
from collections import deque
import matplotlib.pyplot as plt

# Create and configure the BNO sensor connection.  Make sure only ONE of the
# below 'bno = ...' lines is uncommented:
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
pi = pigpio.pi()
bno = BNO055(pi=pi, rst=18, address=0x28, bus=1)
cfilt00 = ComplementaryFilter(alpha=1-0, rollangle=0)
cfilt02 = ComplementaryFilter(alpha=1-0.02, rollangle=0)
cfilt05 = ComplementaryFilter(alpha=1-0.05, rollangle=0)
cfilt10 = ComplementaryFilter(alpha=1-0.1, rollangle=0)
cfilt20 = ComplementaryFilter(alpha=1-0.2, rollangle=0)
# BeagleBone Black configuration with default I2C connection (SCL=P9_19, SDA=P9_20),
# and RST connected to pin P9_12:
#bno = BNO055.BNO055(rst='P9_12')


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

input('Press enter to begin reading BNO055 data, press Ctrl-C to quit...')

cfilt00_rollangle = []
cfilt02_rollangle = []
cfilt05_rollangle = []
cfilt10_rollangle = []
cfilt20_rollangle = []

#gyro_rollangle = []
#cfilt_rollangle = []
#euler_rollangle = []
sim_time = []


t = time.time()
current_time = time.time()
time_start = current_time
period = 0.01
rollangle = 0
count = 0


mx0,my0,mz0 = bno.read_magnetometer()

m0 = (mx0, my0, mz0)
while True: #count < 500:
    count = count + 1
    previous_time = current_time
    current_time = time.time()

    t = t + period
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    #heading, roll, pitch = bno.read_euler()
    gx, gy, gz = bno.read_gyroscope()
    ax, ay, az = bno.read_accelerometer()
    mx, my, mz = bno.read_magnetometer()
    m = (mx,my,mz) #bno.read_magnetometer()
    heading, roll, pitch = bno.read_euler()

    imu_data = [gx,gy,gz,ax,ay,az,mx,my,mz]

    cfilt00.update(imu_data, current_time-previous_time)
    cfilt02.update(imu_data, current_time-previous_time)
    cfilt05.update(imu_data, current_time-previous_time)
    cfilt10.update(imu_data, current_time-previous_time)
    cfilt20.update(imu_data, current_time-previous_time)

    cfilt00_rollangle.append(cfilt00.rollangle)
    cfilt02_rollangle.append(cfilt02.rollangle)
    cfilt05_rollangle.append(cfilt05.rollangle)
    cfilt10_rollangle.append(cfilt10.rollangle)
    cfilt20_rollangle.append(cfilt20.rollangle)

    print(cfilt02.rollangle)

    #print(np.linalg.norm(m))
    #mag_roll = np.arccos(np.dot(m0, m)/(np.linalg.norm(m0)*np.linalg.norm(m)))
    #mag_roll = np.arccos(m[2]/np.linalg.norm(m))
    #print(mag_roll)
    #print('Mx={} My={} Mz={}'.format(mx, my, mz))
    #print('{}, {}'.format(cfilt02.rollangle, np.arctan2(my-my0,mz-mz0)*180/np.pi))

    #print('{}, {}'.format(cfilt.rollangle, cfilt.gyro_only_angle))
    #gyro_rollangle.append(cfilt.gyro_only_angle)
    #cfilt_rollangle.append(cfilt.rollangle)
    #euler_rollangle.append(-1*pitch)
    sim_time.append(current_time)
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    #sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
    #      heading, roll, pitch, sys, gyro, accel, mag))


    #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}'.format(heading, roll, pitch))

    # Other values you can optionally read:
    # Orientation as a quaternion:
    #x,y,z,w = bno.read_quaterion()
    # Sensor temperature in degrees Celsius:
    #temp_c = bno.read_temp()
    # Magnetometer data (in micro-Teslas):
    #x,y,z = bno.read_magnetometer()
    # Gyroscope data (in degrees per second):
    #x,y,z = bno.read_gyroscope()
    # Accelerometer data (in meters per second squared):
    #x,y,z = bno.read_accelerometer()
    # Linear acceleration data (i.e. acceleration from movement, not gravity--
    # returned in meters per second squared):
    #x,y,z = bno.read_linear_acceleration()
    # Gravity acceleration data (i.e. acceleration just from gravity--returned
    # in meters per second squared):
    #x,y,z = bno.read_gravity()
    # Sleep for a second until the next reading.
    time.sleep(max(0, t-time.time()))

plt.rcParams.update({'font.size': 22})
plt.figure()
plt.plot(sim_time, cfilt00_rollangle, label = '0', linewidth=2.0)
plt.plot(sim_time, cfilt02_rollangle, label = '2', linewidth=2.0)
plt.plot(sim_time, cfilt05_rollangle, label = '5', linewidth=2.0)
plt.plot(sim_time, cfilt10_rollangle, label = '10', linewidth=2.0)
plt.plot(sim_time, cfilt20_rollangle, label = '20', linewidth=2.0)

#plt.plot(sim_time, gyro_rollangle, label = 'Gyro Only', linewidth=4.0)
#plt.plot(sim_time, cfilt_rollangle, label = 'Complementary Filter', linewidth=4.0)
#plt.plot(sim_time, euler_rollangle,, label = 'On-Board Roll', linewidth=4.0)
plt.xlabel('Time, [s]')
plt.ylabel('Theta, [deg]')
#plt.ylim(-100,100)
plt.legend()
plt.grid()

plt.show()
