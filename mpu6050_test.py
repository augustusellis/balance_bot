# Script meant for testing the Sparkfun MPU-6050 6DOF accelerometer/gyro board.

import smbus
from mpu6050 import mpu6050
from tcp_client import tcp_client

# Start TCP Client
hostname = '0.0.0.0'
port = 14150
#my_client = tcp_client(hostname, port)

# Default channel is 1, connected to GPIO pins
channel = 1

# MPU-6050 defaults to address 0x68
address = 0x68

reg_write_dac = 0x40 # Currently unnecessary

# Initialize I2C
bus = smbus.SMBus(channel)
# Pin3: SDA
# Pin5: SCL
# Power with 3.3 V

mpu = mpu6050(address)

print(mpu.read_accel_range())
input('Press enter to GO')

while True:
    accel_data = mpu.get_accel_data()
    #my_client.send_msg(accel_data)
    print(accel_data)
    #print(mpu.get_gyro_data())
