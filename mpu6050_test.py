# Script meant for testing the Sparkfun MPU-6050 6DOF accelerometer/gyro board.

import smbus
from mpu6050 import mpu6050

# Default channel is 1, connected to GPIO pins
channel = 1

# MPU-6050 defaults to address asdf
address = 0x68

# What are register addresses?
reg_write_dac = 0x40

# Initialize I2C
bus = smbus.SMBus(channel)

mpu = mpu6050(address)

while True:
    # print(mpu.get_accel_data())
    print(mpu.get_gyro_data())
