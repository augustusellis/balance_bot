#!/usr/bin/env python

import pigpio
from MotorAndEncoder.rotary_encoder import rotary_encoder

class motor:
    """
    Class to drive motor
    """

    def __init__(self, pi, gpio1, gpio2, pwm_pin, dec_pin1, dec_pin2, countsPerRevolution = 227.6*48, encoder=False):
        """
        pi: pigpio pi object
        gpio1: motor pin 1
        gpio2: motor pin 2
        pwm_pin: hardware pwm pin (there are a limited number on the pi)
        """

        self.pi = pi
        self.gpio1 = gpio1
        self.gpio2 = gpio2
        self.pwm_freq = 8000 # this is the maximum allowable frequency
        self.pwm_pin = pwm_pin
        self.dec_pin1 = dec_pin1
        self.dec_pin2 = dec_pin2
        self.dir = None

        if encoder == True:
            self.decoder = rotary_encoder(self.pi, self.dec_pin1, self.dec_pin2, countsPerRevolution)
        else:
            pass
            #self.decoder = rotary_encoder.VirtualDecoder(self.pi, self.dec_pin1, self.dec_pin2, countsPerRevolution)

        self.pi.set_mode(self.gpio1, pigpio.OUTPUT)
        self.pi.set_mode(self.gpio2, pigpio.OUTPUT)

        self.pi.set_PWM_frequency(self.pwm_pin, self.pwm_freq)
        self.pi.set_PWM_range(self.pwm_pin, 10000)

        self.set_duty_cycle(0)
        self.set_direction(1)

        #self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
        #self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

    def get_pos(self):
        return self.decoder.pos

    def set_duty_cycle(self, percent_cycle):
        '''
        sets the duty cycle of the motor
        percent_cycle: -100 to 100 (full speed each direction)
        '''
        # check direction
        new_direction = 0
        if percent_cycle >= 0:
            new_direction = 1
        else:
            new_direction = -1
            percent_cycle = abs(percent_cycle)

        # check duty cycle bounds
        if percent_cycle >= 100:
            percent_cycle = 100
        elif percent_cycle == 0:
            pass
        elif percent_cycle <= 5:
            percent_cycle = 5

        # convert to pwm scale (integer 0 - 1M)
        duty_cycle = int(percent_cycle*100)

        # send new PWM and direction commands
        self.set_direction(new_direction)
        #self.pi.hardware_PWM(self.pwm_pin, self.pwm_freq, duty_cycle)
        self.pi.set_PWM_dutycycle(self.pwm_pin, duty_cycle)

    def set_direction(self, new_direction):
        if new_direction != self.dir:
            if new_direction == 1:
                self.pi.write(self.gpio1, 0)
                self.pi.write(self.gpio2, 1)
                self.dir = 1
                #print("CounterClockwise")
            else:
                self.pi.write(self.gpio1, 1)
                self.pi.write(self.gpio2, 0)
                self.dir = -1
                #print("Clockwise")
