import time
import pigpio
from BNO055 import BNO055
import numpy as np
import pickle

# Create and configure the BNO sensor connection.  Make sure only ONE of the
# below 'bno = ...' lines is uncommented:
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
pi = pigpio.pi()
bno = BNO055(pi=pi, rst=18, address=0x28, bus=1)

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

t = time.time()
current_time = time.time()
time_start = current_time

period = 0.01

count = 0

while count < 3000:
    count = count + 1
    previous_time = current_time
    current_time = time.time()
    t = t + period

    heading, roll, pitch = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(heading, roll, pitch, sys, gyro, accel, mag))
    time.sleep(max(0, t-time.time()))

with open('bno_calibration.cal', 'wb') as f:
    pickle.dump(bno.get_calibration(), f)
    print(bno.get_calibration())
