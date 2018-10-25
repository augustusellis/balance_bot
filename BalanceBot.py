import time
from mpu6050 import mpu6050
from motor import motor
import pigpio # Remember to enable pigpiod

# Define pin numbers:
AI1 = 14
AI2 = 15
pwmA = 18
# Pin3: SDA
# Pin5: SCL

# Initialize Pi
pi = pigpio.pi()

# Initialize MPU
mpu_address = 0x68
mpu = mpu6050(mpu_address) # (Power with 3.3 V)

# Initialize Motors
motor1 = motor(pi,AI1,AI2,pwmA)

'''
motor1.set_duty_cycle(100)
input('Press enter to switch directions')
print('Setting ccw')
motor1.set_direction(1)
print(motor1.dir)
input('Press enter to switch directions')
print('Setting cw')
motor1.set_direction(-1)
print(motor1.dir)
'''
# Run main loop
try:
    while True:
        accel_data = mpu.get_accel_data()
        xval = abs(accel_data['x'])*10
        print(xval)
        motor1.set_duty_cycle( xval )
        #time.sleep(0.01)
except KeyboardInterrupt:
    print('Turning off motors.')
    motor1.set_duty_cycle(0)
