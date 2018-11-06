#!/usr/bin/env python

import pigpio

class decoder:
    """
    Class to decode mechanical rotary encoder pulses.
    """

    def __init__(self, pi, gpioA, gpioB, countsPerRevolution=360):
        """
        Instantiate the class with the pi and gpios connected to
        rotary encoder contacts A and B.  The common contact
        should be connected to ground.
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

        self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):
        """
        Decode the rotary encoder pulse.

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

    def update_position(self, delta_pos):
        self.pos = self.pos + delta_pos

    def get_position(self):
        return self.pos/self.countsPerRevolution*360

    def cancel(self):
        """
        Cancel the rotary encoder decoder.
        """
        self.cbA.cancel()
        self.cbB.cancel()

class VirtualDecoder:
    """
    Virtual Class to Be Used when there is no encoder on the motor.
    """

    def __init__(self, pi, gpioA, gpioB, countsPerRevolution=360):

        self.pi = pi
        self.gpioA = gpioA
        self.gpioB = gpioB

        self.countsPerRevolution = countsPerRevolution

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        self.dir = 1 # Direction is either 1 or -1
        self.pos = 0

    def update_position(self, delta_pos):
        self.pos = self.pos + delta_pos

    def get_position(self):
        return 0

    def cancel(self):
        return 0
