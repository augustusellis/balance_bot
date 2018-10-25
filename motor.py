#!/usr/bin/env python

import pigpio

import rotary_encoder

class motor:
    """
    Class to drive motor.
    """

    def __init__(self, pi, gpio1, gpio2, pwm_pin, dec_pin1=7, dec_pin2=8):
        """
        pi: pigpio pi object
        gpio1: motor pin 1
        gpio2: motor pin 2
        pwm_pin: hardware pwm pin (there are a limitied number on the pi)
        """

        self.pi = pi
        self.gpio1 = gpio1
        self.gpio2 = gpio2
        self.pwm_freq = 125000 # this is the maximum allowable frequency
        self.pwm_pin = pwm_pin
        self.dec_pin1 = dec_pin1
        self.dec_pin2 = dec_pin2
        self.dir = None

        self.decoder = rotary_encoder.decoder(self.pi, self.dec_pin1, self.dec_pin2)

        self.pi.set_mode(self.gpio1, pigpio.OUTPUT)
        self.pi.set_mode(self.gpio2, pigpio.OUTPUT)

        self.set_duty_cycle(0)
        self.set_direction(1)

        #self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
        #self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

    def get_pos(self):
        return self.decoder.pos

    def set_duty_cycle(self, percent_cycle):
        duty_cycle = int(percent_cycle*10000)
        print(duty_cycle)
        if duty_cycle >= 1000000:
            duty_cycle = 1000000
        self.pi.hardware_PWM(self.pwm_pin, self.pwm_freq, duty_cycle)

    def set_direction(self, new_direction):
        if new_direction != self.dir:
            if new_direction == 1:
                self.pi.write(self.gpio1, 0)
                self.pi.write(self.gpio2, 1)
                self.dir = 1
                print("CounterClockwise")
            else:
                self.pi.write(self.gpio1, 1)
                self.pi.write(self.gpio2, 0)
                self.dir = -1
                print("Clockwise")


#if __name__ == "__main__":
