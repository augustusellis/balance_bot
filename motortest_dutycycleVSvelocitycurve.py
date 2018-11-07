# Script for testing the motor driver as well as the rotary encoder.

import RPi.GPIO as GPIO
import time
import pigpio           # Remember to enable pigpiod
import rotary_encoder
#import matplotlib.pyplot as plt

# Define Pins:
pwma_pin = 18   # Pi pin 12
AI1 = 3         # Pi pin 5
AI2 = 4         # Pi pin 7
encA = 7
encB = 8

# Setup Pins:
#GPIO.cleanup()
GPIO.setmode(GPIO.BCM)          # Broadcom pin-numbering scheme
#GPIO.setup(pwma_pin, GPIO.OUT)
#pwma = GPIO.PWM(pwma_pin, pwm_freq)  # Initialize PWM on pwma_pin 500Hz frequency
GPIO.setup(AI1, GPIO.OUT)
GPIO.setup(AI2, GPIO.OUT)
#GPIO.setup(encA, GPIO.IN)
#GPIO.setup(encB, GPIO.IN)

# Set Motor Driver to run clockwise:
GPIO.output(AI1, GPIO.LOW)
GPIO.output(AI2, GPIO.HIGH)

#dc = 100          # Duty Cycle
#pwma.start(dc)

pwm_freq = 10000
pi = pigpio.pi()
pi.hardware_PWM(pwma_pin,pwm_freq,1000000)
decoder = rotary_encoder.decoder(pi, 7, 8)


#aval = GPIO.input(encA)
#bval = GPIO.input(encB)
count = 0
position = 0
error = (0,0)
eP = 0
eI = 0
eD = 0
kP = 1
kI = 0
kD = 100
referencePosition = 2000
previousError = 0
duty_cycle = 0
previousPosition = decoder.pos
currentPosition = previousPosition
pos = []
dc = []
vel = []
try:
    for duty_cycle in range(1000001):
        previousPosition = currentPosition
        currentPosition = decoder.pos
        pos.append(currentPosition)
        dc.append(duty_cycle)
        '''
        if dir == 1:
            GPIO.output(AI1, GPIO.LOW)
            GPIO.output(AI2, GPIO.HIGH)
        else:
            GPIO.output(AI1, GPIO.HIGH)
            GPIO.output(AI2, GPIO.LOW)
        '''
        pi.hardware_PWM(pwma_pin, pwm_freq, duty_cycle)
    pi.hardware_PWM(pwma_pin,pwm_freq,0)
    #pwma.stop()
    GPIO.cleanup()
    decoder.cancel()
    pi.stop()
    with open('dcVSPos.csv', 'w') as filehandle:
        filehandle.writelines("{}, {}\n".format(dataPoint[0], dataPoint[1]) for dataPoint in zip(dc, pos))
    #print(dc)
    #print(pos)
    #plt.plot(dc, pos)
    #plt.show()
    '''
    #dc = (dc+0.0001)%100
    #pwma.ChangeDutyCycle(dc)
    if (GPIO.input(encA) != aval):
        count = count+1
        aval = GPIO.input(encA)
        print("{} A: {}, B: {}".format(count, aval, bval))
    if (GPIO.input(encB) != bval):
        count = count+1
        bval = GPIO.input(encB)
        print("{} A: {}, B: {}".format(count, aval, bval))
    '''
except KeyboardInterrupt:
    pi.hardware_PWM(pwma_pin,pwm_freq,0)
    #pwma.stop()
    GPIO.cleanup()
    decoder.cancel()
    pi.stop()
