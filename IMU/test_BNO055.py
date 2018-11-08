import time
import pigpio
from BNO055 import BNO055


# Create and configure the BNO sensor connection.  Make sure only ONE of the
# below 'bno = ...' lines is uncommented:
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
pi = pigpio.pi()
bno = BNO055(pi=pi, rst=18, address=0x28, bus=1)
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
while True:
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    #heading, roll, pitch = bno.read_euler()
    gx, gy, gz = bno.read_gyroscope()
    ax, ay, az = bno.read_accelerometer()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    #sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
    #      heading, roll, pitch, sys, gyro, accel, mag))

    print('Gx={0:0.2F} Gy={1:0.2F} Gz={2:0.2F} Ax={3:0.2F} Ay={4:0.2F} Az={5:0.2F}'.format(gx, gy, gz, ax, ay, az))
    #print('Ax={0:0.2F} Ay={1:0.2F} Az={2:0.2F}'.format(ax, ay, az))
    #print('Gx={:6d} Gy={:6d} Gz={:6d} Ax={:6d} Ay={:6d} Az={:6d}'.format(gx, gy, gz, ax, ay, az))

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
    time.sleep(0.1)
