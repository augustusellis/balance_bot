#!/usr/bin/env python

import pigpio

class rotary_encoder:
    """
    Class to decode mechanical rotary encoder pulses.
    POSITIVE POSITION = CCW
    """

    def __init__(self, pi, gpioA, gpioB, countsPerRevolution=46.85*12):
        """
        Instantiate the class with the pi and gpios connected to
        rotary encoder contacts A and B.  The common contact
        should be connected to ground.
        :param pi:                  (pigpio.pi object) pigpio raspberry pi reference variable
        :param gpioA:               (int) encoder channel A pin number
        :param gpioB:               (int) enocder channel B pin number
        :param countsPerRevolution: (float) ratio of quad encoder counts to each revolution of the output shaft
        """

        self.pi = pi
        self.gpioA = gpioA
        self.gpioB = gpioB

        self.countsPerRevolution = countsPerRevolution

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        self.dir = 1 # Direction is either 1 or -1
        self.pos = 0

        self.pi.set_mode(gpioA, pigpio.INPUT)
        self.pi.set_mode(gpioB, pigpio.INPUT)

        self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(gpioA, pigpio.RISING_EDGE, self._pulseA)
        self.cbB = self.pi.callback(gpioB, pigpio.RISING_EDGE, self._pulseB)

    def _pulse(self, gpio, level, tick):
        """
        Decode the rotary encoder pulse.
        :param gpio:        (int) triggered gpio number
        :param level:       (int) triggered gpio level (1 or 0)
        :param tick:        (int) current system up-time in microseconds, wraps ~79.1 minutes

                  +---------+         +---------+      0
                  |         |         |         |
        A         |         |         |         |
                  |         |         |         |
        +---------+         +---------+         +----- 1

            +---------+         +---------+            0
            |         |         |         |
        B   |         |         |         |
            |         |         |         |
        ----+         +---------+         +---------+  1
        """
        a_prev = self.levA
        b_prev = self.levB

        if gpio == self.gpioA:
            self.lastGpio = self.gpioA
            self.levA = level
        else:
            self.lastGpio = self.gpioB
            self.levB = level

        if (not self.levA and not b_prev) or (self.levA and b_prev):
            self.dir = 1
        else:
            self.dir = -1

        self.update_position(self.dir)

    def _pulseA(self, gpio, level, tick):
        '''
        GPIO callback for pin A
        '''
        self.update_position(1)

    def _pulseB(self, gpio, level, tick):
        '''
        GPIO callback for pin B
        '''
        self.update_position(-1)

    def update_position(self, delta_pos):
        '''
        Update raw position from gpio callbacks. 
        '''
        self.pos = self.pos + delta_pos

    def get_position(self):
        '''
        Returns encoder position in revolutions (float)
        '''
        return self.pos/self.countsPerRevolution

    def cancel(self):
        """
        Cancel the rotary encoder decoder.
        """
        self.cbA.cancel()
        self.cbB.cancel()
