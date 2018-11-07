# Script for testing the motor driver as well as the rotary encoder.

import RPi.GPIO as GPIO
import time
import pigpio           # Remember to enable pigpiod
import rotary_encoder

# Define pipgio position and callback:
pos = 0
def decoder_callback(way):
   global pos
   pos += way
   print("pos={}".format(pos))

# Define Pins:
pwma_pin = 18   # Pi pin 12
AI1 = 3         # Pi pin 5
AI2 = 4         # Pi pin 7
encA = 7
encB = 8

# Setup Pins:
#GPIO.cleanup()
GPIO.setmode(GPIO.BCM)          # Broadcom pin-numbering scheme
GPIO.setup(pwma_pin, GPIO.OUT)
pwma = GPIO.PWM(pwma_pin, 500)  # Initialize PWM on pwma_pin 500Hz frequency
GPIO.setup(AI1, GPIO.OUT)
GPIO.setup(AI2, GPIO.OUT)
GPIO.setup(encA, GPIO.IN)
GPIO.setup(encB, GPIO.IN)

# Set Motor Driver to run clockwise:
GPIO.output(AI1, GPIO.LOW)
GPIO.output(AI2, GPIO.HIGH)

dc = 15          # Duty Cycle
pwma.start(dc)


pi = pigpio.pi()
decoder = rotary_encoder.decoder(pi, 7, 8, decoder_callback)


#aval = GPIO.input(encA)
#bval = GPIO.input(encB)
#count = 0
try:
    while True:
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
    pwma.stop()
    GPIO.cleanup()
    decoder.cancel()
    pi.stop()
