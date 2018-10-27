import time
from mpu6050 import mpu6050
from motor import motor
import pigpio # Remember to enable pigpiod
import numpy as np

# Define pin numbers:
AI1 = 15
AI2 = 14
pwmA = 18

BI1 = 23
BI2 = 24
pwmB = 19
# Pin3: SDA
# Pin5: SCL

# Initialize Pi
pi = pigpio.pi()

# Initialize MPU
mpu_address = 0x68
mpu = mpu6050(mpu_address) # (Power with 3.3 V)

# Initialize Motors
motor2 = motor(pi,BI1,BI2,pwmB, encoder=False)
motor1 = motor(pi,AI1,AI2,pwmA, encoder=False)


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
    val_ref = 0.45 #9.55
    yref = 0.45
    error = 0
    errorPrevious = 0
    eP = 0
    eI = 0
    eD = 0

    kP = 20
    kI = 0
    kD = 0

    while True:
        accel_data = mpu.get_accel_data()
        val = (accel_data['y'])

        error = val_ref - val

        #error = -1*error*np.sign(accel_data['y']-yref)

        eP = error
        eI = eI + error
        eD = error - errorPrevious

        errorPrevious = error

        u1 = kP*eP + kI*eI + kD*eD
        u2 = u1
        print('{:+.4f}, {:+.4f}'.format(error, u1))
        motor1.set_duty_cycle(u1)
        motor2.set_duty_cycle(u2)

except KeyboardInterrupt:
    print('Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)


except IOError:
    print('IOERROR: Turning off motors.')
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)
