# Script meant for testing the Sparkfun MPU-6050 6DOF accelerometer/gyro board.

import smbus
from mpu6050 import mpu6050
from tcp_client import tcp_client

# Start TCP Client
hostname = '0.0.0.0'
port = 14150
my_client = tcp_client(hostname, port)

# Default channel is 1, connected to GPIO pins
channel = 1

# MPU-6050 defaults to address asdf
address = 0x68

reg_write_dac = 0x40 # Currently unnecessary

# Initialize I2C
bus = smbus.SMBus(channel)

mpu = mpu6050(address)


while True:
    accel_data = mpu.get_accel_data()
    my_client.send_msg(accel_data)
    print(accel_data)
    #print(mpu.get_gyro_data())
