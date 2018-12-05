#!/usr/bin/env python

import pigpio
from MotorAndEncoder.rotary_encoder import rotary_encoder

class motor:
    """
    Class to drive DC motor
    """

    def __init__(self, pi, gpio1, gpio2, pwm_pin, dec_pin1, dec_pin2, countsPerRevolution = 46.85*12, encoder=False):
        """
        Initialize the class with the given parameters.
        :param pi:                  (pigpio.pi object) pigpio raspberry pi reference variable
        :param gpio1:               (int) direction gpio pin number
        :param gpio2:               (int) direction gpio pin number
        :param pwm_pin:             (int) motor pwm pin number (broadcom)
        :param dec_pin1:            (int) rotary encoder pin A
        :param dec_pin2:            (int) rotary encoder pin B
        :param countsPerRevolution: (float) ratio of quad encoder counts to each revolution of the output shaft
        :param encoder:             (bool) True if the motor has an encoder, False if it does not
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

        self.pi.set_mode(self.gpio1, pigpio.OUTPUT)
        self.pi.set_mode(self.gpio2, pigpio.OUTPUT)

        self.pi.set_PWM_frequency(self.pwm_pin, self.pwm_freq)
        self.pi.set_PWM_range(self.pwm_pin, 10000)

        self.set_duty_cycle(0)
        self.set_direction(1)

    def get_pos(self):
        '''
        Returns the motor angular position as determined by the quadrature encoder
        '''
        return self.decoder.get_position()

    def set_duty_cycle(self, percent_cycle):
        '''
        Sets the duty cycle of the motor.
        :param percent_cycle: (float) pwm duty cycle, -100 to 100 (full speed each direction)
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
        elif percent_cycle <= 10:
            percent_cycle = 10

        # convert to pwm scale (integer 0 - 1M)
        duty_cycle = int(percent_cycle*100)

        # send new PWM and direction commands
        self.set_direction(new_direction)
        #self.pi.hardware_PWM(self.pwm_pin, self.pwm_freq, duty_cycle)
        self.pi.set_PWM_dutycycle(self.pwm_pin, duty_cycle)

    def set_direction(self, new_direction):
        '''
        Sets the rotation direction of the motor.
        :param new_directione: (int) motor direction, -1 (CCW), 1 (CW)
        '''
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
